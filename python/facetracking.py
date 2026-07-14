"""
Face Tracker - GUI
=====================
Interface Tkinter para o sistema de rastreamento de ROSTO (cvzone.FaceDetector) + ESP32-CAM.
ARQUITETURA:

  Thread 1 (CameraStream._update)   -> só captura frames do stream HTTP,
            sempre mantém o frame MAIS RECENTE em memória.

  Thread 2 (FaceTrackerWorker.run)  -> pega o frame mais recente, roda a
            detecção de rosto (cvzone.FaceDetector), desenha as anotações,
            envia UDP pro ESP32, e guarda o resultado anotado em
            self.display_frame. Roda em loop livre, na velocidade que
            conseguir, SEM esperar a GUI.

  Thread principal (Tkinter)        -> só faz dois trabalhos, ambos leves:
            1) a cada ~33ms (30 FPS de display), pega display_frame e
               desenha no Label da câmera.
            2) lê/escreve os campos de IP, porta e variáveis de PID.

Como a detecção roda numa thread própria e a GUI só "olha" o resultado através
de um lock (sem bloquear a thread de tracking), o desempenho da detecção não é
afetado pela interface.
"""
import socket
import threading
import time
import sys
import os
import csv
import tkinter as tk
from tkinter import ttk
import cv2
import numpy as np
from PIL import Image, ImageTk
from cvzone.FaceDetectionModule import FaceDetector

# ------------------------------------------------------------
# CONFIGURAÇÃO PADRÃO (pode ser alterada na própria interface)
# ------------------------------------------------------------
DEFAULT_ESP32_IP = "192.168.1.107"
DEFAULT_ESP32_PORT = 4210
DEFAULT_STREAM_URL = "http://esp32cam.local:81/stream"

CAM_DISPLAY_W = 640
CAM_DISPLAY_H = 480
GUI_REFRESH_MS = 33  # ~30 FPS para o display (a thread de tracking roda livre, independente disso)

# --- Parâmetros de performance da detecção ---
DETECT_EVERY_N_FRAMES = 3   # roda a detecção pesada a cada N frames
DETECT_SCALE = 0.5          # detecta numa cópia reduzida do frame (mais rápido); 0.5 = metade da resolução

# Variáveis de PID enviadas ao ESP32 via UDP (pacote "CFG,...").
# Tupla = (chave usada no protocolo / igual ao firmware, rótulo exibido na GUI, valor padrão)
PID_VARS = [
    ("kpx", "KpX", "0.001"),
    ("kix", "KiX", "0.00001"),
    ("kdx", "KdX", "0.0001"),
    ("kpy", "KpY", "0.001"),
    ("kiy", "KiY", "0.00001"),
    ("kdy", "KdY", "0.0001"),
]


def resource_path(relative_path):
    """ Obtém o caminho absoluto para o recurso, funciona para dev e para PyInstaller """
    try:
        # PyInstaller cria uma pasta temporária e armazena o caminho em _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")
    return os.path.join(base_path, relative_path)


# NOTE: --------------------------------------------------------------------------
#   pasta de fácil acesso para salvar os logs de dados.
#   Importante: NÃO usamos resource_path()/_MEIPASS aqui, porque quando o programa
#   está empacotado (PyInstaller) essa pasta é temporária e é APAGADA quando o
#   programa fecha - qualquer log salvo lá se perderia. Em vez disso, salvamos
#   dentro de "Documentos", que existe tanto em modo desenvolvimento quanto no
#   executável final, e é fácil do usuário encontrar depois.
def get_log_dir():
    documentos = os.path.join(os.path.expanduser("~"), "Documents")
    if not os.path.isdir(documentos):
        # fallback (ex.: sistemas sem pasta "Documents" nesse caminho) ->
        # usa a pasta pessoal do usuário
        documentos = os.path.expanduser("~")
    log_dir = os.path.join(documentos, "FaceTracker_Logs")
    os.makedirs(log_dir, exist_ok=True)
    return log_dir


# ------------------------------------------------------------
# THREAD DE CAPTURA (idêntica ao RescueTracker)
# ------------------------------------------------------------
class CameraStream:
    def __init__(self, src):
        self.src = src
        self.cap = None
        self.ret = False
        self.frame = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.error = None
        # --- FPS real da câmera (medido aqui, onde os frames realmente chegam) ---
        self.fps = 0.0
        self._fps_alpha = 0.1
        self._last_frame_time = None

    def start(self):
        self.cap = cv2.VideoCapture(self.src, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.error = "Erro ao acessar ESP32-CAM (stream não abriu)."
            return False

        self.ret, self.frame = self.cap.read()
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        return True

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue

            now = time.time()
            with self.lock:
                self.ret = ret
                self.frame = frame

                # calcula o FPS com base no intervalo entre frames REAIS
                # (essa thread só itera quando cap.read() devolve um frame novo)
                if self._last_frame_time is not None:
                    dt = now - self._last_frame_time
                    if dt > 0:
                        fps_instantaneo = 1.0 / dt
                        if self.fps == 0.0:
                            self.fps = fps_instantaneo
                        else:
                            self.fps = (self._fps_alpha * fps_instantaneo) + ((1 - self._fps_alpha) * self.fps)
                self._last_frame_time = now

    def get_fps(self):
        with self.lock:
            return self.fps

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return self.ret, self.frame.copy()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2)
        if self.cap is not None:
            self.cap.release()


# ------------------------------------------------------------
# THREAD DE PROCESSAMENTO - Detecção de rosto (cvzone) + envio UDP
# ------------------------------------------------------------
class FaceTrackerWorker:
    def __init__(self, cam: CameraStream, get_esp_target, log_callback):
        self.cam = cam
        self.get_esp_target = get_esp_target  # função -> (ip, porta) atuais, lidos da GUI
        self.log_callback = log_callback       # função para mandar mensagens pro Monitor/Erros da GUI

        # modelSelection=0 = modelo "short range" do mediapipe (mais rápido,
        # ideal para rosto relativamente próximo da câmera, como é o caso aqui)
        self.detector = FaceDetector(minDetectionCon=0.5, modelSelection=0)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # --- Cache de detecção ---
        # Guarda o último bbox conhecido para ser reaproveitado nos frames em
        # que a detecção pesada é "pulada" (ver DETECT_EVERY_N_FRAMES).
        self.frame_count = 0
        self.last_bboxs = None

        self.running = False
        self.thread = None

        # Frame anotado mais recente, pronto para exibição (protegido por lock)
        self.display_lock = threading.Lock()
        self.display_frame = None

        # Estatísticas para exibir na GUI (erroX, erroY, fps, alvo)
        self.stats_lock = threading.Lock()
        self.stats = {"erroX": 0, "erroY": 0, "fps": 0.0, "alvo": False}

        # --------------------- GRAVAÇÃO DE DADOS PARA GRÁFICO ---------------------
        # Salva erroX, erroY e fps ao longo do tempo em um CSV
        self.data_log_lock = threading.Lock()   # protege o acesso ao arquivo/writer (chamado de 2 threads)
        self.data_log_active = False
        self.data_log_path = None
        self.data_log_file = None
        self.data_log_writer = None
        self.data_log_start = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2)

        # garante que a gravação seja fechada com segurança caso o usuário desconecte sem clicar em "Parar Gravação" antes.
        if self.data_log_active:
            self.stop_recording()

    def get_display_frame(self):
        with self.display_lock:
            if self.display_frame is None:
                return None
            return self.display_frame.copy()

    # ---------------- controle de gravação (chamado pela GUI) ----------------
    def start_recording(self):
        """Cria um novo arquivo CSV e começa a gravar os dados. Retorna o caminho do arquivo,
        ou None se não foi possível criar o arquivo."""
        with self.data_log_lock:
            if self.data_log_active:
                return self.data_log_path  # já está gravando

            log_dir = get_log_dir()
            filename = f"tracking_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            path = os.path.join(log_dir, filename)

            try:
                arquivo = open(path, "w", newline="")
            except OSError as e:
                self.log_callback("erro", f"Não foi possível criar o arquivo de log: {e}")
                return None

            writer = csv.writer(arquivo)
            writer.writerow(["timestamp", "tempo_s", "erroX", "erroY", "fps", "alvo"])

            self.data_log_file = arquivo
            self.data_log_writer = writer
            self.data_log_path = path
            self.data_log_start = time.time()
            self.data_log_active = True

        self.log_callback("info", f"Gravação iniciada. Salvando dados em: {path}")
        return path

    def stop_recording(self):
        """Fecha o arquivo CSV atual, se houver gravação em andamento."""
        with self.data_log_lock:
            if not self.data_log_active:
                return
            self.data_log_active = False
            path = self.data_log_path
            try:
                if self.data_log_file is not None:
                    self.data_log_file.flush()
                    self.data_log_file.close()
            except OSError:
                pass
            self.data_log_file = None
            self.data_log_writer = None

        self.log_callback("info", f"Gravação finalizada. Arquivo salvo em: {path}")

    def _log_data(self, stats):
        """Escreve uma linha do CSV com os dados do frame atual (só faz algo se estiver gravando)."""
        with self.data_log_lock:
            if not self.data_log_active or self.data_log_writer is None:
                return
            writer = self.data_log_writer
            arquivo = self.data_log_file
            inicio = self.data_log_start

        tempo_s = time.time() - inicio
        agora = time.strftime("%Y-%m-%d %H:%M:%S")

        try:
            writer.writerow([
                agora,
                f"{tempo_s:.3f}",
                stats["erroX"],
                stats["erroY"],
                f"{stats['fps']:.2f}",
                stats["alvo"],
            ])
            # Flush periódico (não a cada frame, pra não pesar no desempenho da thread de tracking)
            if self.frame_count % 10 == 0:
                arquivo.flush()
        except (OSError, ValueError) as e:
            self.log_callback("erro", f"Falha ao escrever log de dados: {e}")
            with self.data_log_lock:
                self.data_log_active = False
    # ----------------------------------------------------------------------------------------------

    def _detectar_rosto(self, img):
        """
        Roda o detector de rosto numa cópia REDUZIDA do frame (DETECT_SCALE) e
        devolve os bboxs já reescalados para as coordenadas do frame original.
        Detectar em resolução menor é o que mais pesa no custo de CPU do
        mediapipe/cvzone — reduzir a imagem antes de detectar costuma dar o
        maior ganho de fluidez, mais até que pular frames.
        """
        small = cv2.resize(img, (0, 0), fx=DETECT_SCALE, fy=DETECT_SCALE)
        _, bboxs_small = self.detector.findFaces(small, draw=False)

        if not bboxs_small:
            return []

        inv_scale = 1.0 / DETECT_SCALE
        bboxs = []
        for b in bboxs_small:
            x, y, w, h = b["bbox"]
            x, y, w, h = int(x * inv_scale), int(y * inv_scale), int(w * inv_scale), int(h * inv_scale)
            cx, cy = int(b["center"][0] * inv_scale), int(b["center"][1] * inv_scale)
            bboxs.append({"bbox": (x, y, w, h), "center": (cx, cy), "score": b.get("score")})
        return bboxs

    def run(self):
        try:
            while self.running:
                frame_inicio = time.time()

                success, img = self.cam.read()
                if not success or img is None:
                    continue

                hs, ws, _ = img.shape
                centerX, centerY = ws // 2, hs // 2

                self.frame_count += 1

                # --- Detecção de rosto: roda pesada só a cada N frames, reaproveitando o último bbox
                #     conhecido nos frames "pulados". O envio UDP continua
                #     acontecendo em TODO frame, então o controle PID no ESP32
                #     não fica "lento" — só a detecção em si é amortizada. ---
                if self.frame_count % DETECT_EVERY_N_FRAMES == 0 or self.last_bboxs is None:
                    self.last_bboxs = self._detectar_rosto(img)
                bboxs = self.last_bboxs

                # FPS real da câmera, medido na CameraStream (não no loop deste worker)
                fps_camera = self.cam.get_fps()

                if bboxs:
                    fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
                    x, y, w, h = bboxs[0]["bbox"]

                    erroX = fx - centerX
                    erroY = fy - centerY

                    esp_ip, esp_port = self.get_esp_target()
                    message = f"{erroX},{erroY}"
                    try:
                        self.sock.sendto(message.encode(), (esp_ip, esp_port))
                    except OSError as e:
                        self.log_callback("erro", f"Falha ao enviar UDP: {e}")

                    # --- DESENHOS COM ALVO ---
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
                    cv2.putText(img, f"[{fx}, {fy}]", (fx + 15, fy - 15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
                    cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)
                    cv2.line(img, (fx, 0), (fx, hs), (0, 0, 0), 2)
                    cv2.putText(img, "ALVO NA MIRA", (ws - 300, 40), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 3)
                    cv2.putText(img, f"ErroX: {erroX}", (20, 80), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 0), 2)
                    cv2.putText(img, f"ErroY: {erroY}", (20, 120), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 0), 2)

                    with self.stats_lock:
                        self.stats = {"erroX": erroX, "erroY": erroY, "fps": fps_camera, "alvo": True}
                else:
                    # --- DESENHOS SEM ALVO (mira no centro) ---
                    cv2.putText(img, "SEM ALVO", (ws - 250, 40), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 3)
                    cv2.circle(img, (centerX, centerY), 80, (0, 0, 255), 2)
                    cv2.circle(img, (centerX, centerY), 15, (0, 0, 255), cv2.FILLED)
                    cv2.line(img, (0, centerY), (ws, centerY), (0, 0, 0), 2)
                    cv2.line(img, (centerX, 0), (centerX, hs), (0, 0, 0), 2)

                    with self.stats_lock:
                        self.stats = {"erroX": 0, "erroY": 0, "fps": fps_camera, "alvo": False}

                # grava a linha de log deste frame (só escreve algo se
                # a gravação estiver ativa - ver start_recording()/stop_recording())
                self._log_data(self.stats)

                cv2.putText(img, f"FPS: {fps_camera:.1f}", (20, 40), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 2)

                with self.display_lock:
                    self.display_frame = img

        except Exception as e:
            self.log_callback("erro", f"Thread de tracking parou: {e}")


# ------------------------------------------------------------
# INTERFACE GRÁFICA
# ------------------------------------------------------------
class FaceTrackerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Face Tracker | ESP32-CAM Controller")
        try:
            self.root.iconbitmap(resource_path("python/logo_if.ico"))
        except Exception:
            pass  # ícone é opcional; não trava a aplicação se o arquivo não existir
        self.root.geometry("1000x680")
        self.root.minsize(900, 620)

        self.cam = None
        self.worker = None
        self.connected = False
        self.recording = False  # estado do botão de gravação

        # Socket dedicado ao envio de configuração PID — separado do socket
        # de tracking (que só existe dentro do FaceTrackerWorker, criado ao
        # conectar). Assim, dá para enviar configuração mesmo se a câmera
        # ainda não estiver conectada.
        self._cfg_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._build_layout()

    # -------------------- LAYOUT --------------------
    def _build_layout(self):
        style = ttk.Style()
        style.theme_use("default")
        style.configure("Connect.TButton", background="#90EE90")
        style.configure("Disconnect.TButton", background="#FFB3B3")

        # ---- Linha 1: câmera (esquerda) + painel de conexão/variáveis (direita) ----
        top_frame = tk.Frame(self.root)
        top_frame.pack(fill="both", expand=True, padx=8, pady=8)

        # --- Painel da câmera ---
        cam_frame = tk.LabelFrame(top_frame, text="Câmera (ESP32-CAM)")
        cam_frame.pack(side="left", fill="both", expand=True, padx=(0, 6))

        self.cam_label = tk.Label(
            cam_frame, text="Sem sinal", bg="black", fg="white",
            width=CAM_DISPLAY_W // 8, height=CAM_DISPLAY_H // 16,
        )
        self.cam_label.pack(fill="both", expand=True, padx=4, pady=4)

        # --- Painel lateral: tabela de configuração ---
        side_frame = tk.Frame(top_frame, width=340)
        side_frame.pack(side="right", fill="y")

        config_table = tk.Frame(side_frame, highlightbackground="black", highlightthickness=1)
        config_table.pack(fill="x", pady=(0, 8))

        self._add_row(config_table, 0, "IP ESP32:", default=DEFAULT_ESP32_IP, attr="ip_entry")
        self._add_row(config_table, 1, "Porta UDP:", default=str(DEFAULT_ESP32_PORT), attr="port_entry")
        self._add_row(config_table, 2, "Stream URL:", default=DEFAULT_STREAM_URL, attr="stream_entry")

        # Botão Conectar / Desconectar
        self.connect_btn = tk.Button(
            config_table, text="Conectar", bg="#90EE90",
            command=self.toggle_connection,
        )
        self.connect_btn.grid(row=3, column=0, columnspan=2, sticky="nsew", padx=1, pady=1)
        config_table.grid_columnconfigure(0, weight=0)
        config_table.grid_columnconfigure(1, weight=1)

        # --- Tabela de variáveis PID enviadas ao ESP32 via UDP ---
        vars_frame = tk.LabelFrame(side_frame, text="Variáveis PID (ESP32)")
        vars_frame.pack(fill="x", pady=(0, 8))

        self.vars_table = tk.Frame(vars_frame)
        self.vars_table.pack(fill="x", padx=4, pady=4)

        # self.pid_entries: chave do protocolo -> Entry correspondente
        self.pid_entries = {}
        for row_idx, (chave, rotulo, valor_padrao) in enumerate(PID_VARS):
            tk.Label(self.vars_table, text=rotulo, anchor="w", width=18).grid(
                row=row_idx, column=0, padx=2, pady=1, sticky="w"
            )
            entry = tk.Entry(self.vars_table, width=10)
            entry.insert(0, valor_padrao)
            entry.grid(row=row_idx, column=1, padx=2, pady=1, sticky="we")
            self.pid_entries[chave] = entry

        self.vars_table.grid_columnconfigure(1, weight=1)

        btns_frame = tk.Frame(vars_frame)
        btns_frame.pack(fill="x", padx=4, pady=(0, 4))
        tk.Button(
            btns_frame, text="Restaurar Padrões", bg="#E6ADAD", command=self._restore_pid_defaults
        ).pack(side="left", expand=True, fill="x", padx=2)
        tk.Button(
            btns_frame, text="Enviar ao ESP32", bg="#ADD8E6", command=self.send_variables
        ).pack(side="left", expand=True, fill="x", padx=2)

        # --- Estatísticas de rastreamento em tempo real ---
        stats_frame = tk.LabelFrame(side_frame, text="Status do Rastreamento")
        stats_frame.pack(fill="x", pady=(0, 8))

        self.stats_labels = {}
        for i, key in enumerate(["Alvo", "ErroX", "ErroY", "FPS"]):
            tk.Label(stats_frame, text=f"{key}:", anchor="w", width=12).grid(row=i, column=0, sticky="w", padx=4, pady=2)
            lbl = tk.Label(stats_frame, text="--", anchor="w", fg="#0a6b0a", font=("Consolas", 10, "bold"))
            lbl.grid(row=i, column=1, sticky="w", padx=4, pady=2)
            self.stats_labels[key] = lbl

        # --------------------- PAINEL DE GRAVAÇÃO DE DADOS ---------------------
        # Botão para iniciar/parar a gravação dos dados (erroX, erroY, fps) em CSV
        record_frame = tk.LabelFrame(side_frame, text="Gravação de Dados (gráfico)")
        record_frame.pack(fill="x")

        self.record_btn = tk.Button(
            record_frame, text="Iniciar Gravação", bg="#FFD700",
            command=self.toggle_recording, state="disabled",
        )
        self.record_btn.pack(fill="x", padx=4, pady=(4, 2))

        self.record_status_label = tk.Label(
            record_frame, text="Não gravando.", anchor="w", fg="#555555",
            wraplength=300, justify="left",
        )
        self.record_status_label.pack(fill="x", padx=4, pady=(0, 4))

        # ---- Linha 2: Monitor / Erros ----
        bottom_frame = tk.Frame(self.root)
        bottom_frame.pack(fill="both", expand=False, padx=8, pady=(0, 8))

        monitor_frame = tk.LabelFrame(bottom_frame, text="Monitor:")
        monitor_frame.pack(side="left", fill="both", expand=True, padx=(0, 4))
        self.monitor_text = tk.Text(monitor_frame, height=8, state="disabled", bg="white")
        self.monitor_text.pack(fill="both", expand=True, padx=2, pady=2)

        erros_frame = tk.LabelFrame(bottom_frame, text="Erros:")
        erros_frame.pack(side="left", fill="both", expand=True, padx=(4, 0))
        self.erros_text = tk.Text(erros_frame, height=8, state="disabled", bg="white", fg="#a30000")
        self.erros_text.pack(fill="both", expand=True, padx=2, pady=2)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _add_row(self, parent, row, label_text, default, attr):
        tk.Label(parent, text=label_text, anchor="w").grid(row=row, column=0, sticky="nsew", padx=4, pady=2)
        entry = tk.Entry(parent)
        entry.insert(0, default)
        entry.grid(row=row, column=1, sticky="nsew", padx=4, pady=2)
        setattr(self, attr, entry)

    def _restore_pid_defaults(self):
        """Repõe os valores padrão (os mesmos do firmware) em todos os campos PID."""
        for chave, _rotulo, valor_padrao in PID_VARS:
            entry = self.pid_entries[chave]
            entry.delete(0, "end")
            entry.insert(0, valor_padrao)

    # -------------------- LOG HELPERS --------------------
    def log(self, kind, msg):
        """Thread-safe: agenda a escrita no widget de texto na thread principal do Tkinter."""
        self.root.after(0, self._log_ui, kind, msg)

    def _log_ui(self, kind, msg):
        target = self.monitor_text if kind == "info" else self.erros_text
        target.config(state="normal")
        timestamp = time.strftime("%H:%M:%S")
        target.insert("end", f"[{timestamp}] {msg}\n")
        target.see("end")
        target.config(state="disabled")

    # -------------------- CONEXÃO --------------------
    def toggle_connection(self):
        if not self.connected:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        stream_url = self.stream_entry.get().strip()
        self.log("info", f"Conectando ao stream: {stream_url}")

        self.cam = CameraStream(stream_url)
        ok = self.cam.start()
        if not ok:
            self.log("erro", self.cam.error or "Falha ao conectar na câmera.")
            return

        self.log("info", "Câmera conectada. Iniciando detecção de rosto...")
        self.worker = FaceTrackerWorker(self.cam, self.get_esp_target, self.log)
        self.worker.start()
        self._on_connected()

    def _on_connected(self):
        self.connected = True
        self.connect_btn.config(text="Desconectar", bg="#FFB3B3", state="normal")
        self.log("info", "Rastreamento iniciado.")
        self.record_btn.config(state="normal")  # libera o botão de gravação
        self._schedule_gui_update()

    def disconnect(self):
        # se estiver gravando, encerra a gravação antes de desconectar
        if self.recording:
            self.toggle_recording()

        self.connected = False
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
        if self.cam is not None:
            self.cam.stop()
            self.cam = None
        self.connect_btn.config(text="Conectar", bg="#90EE90")
        self.cam_label.config(image="", text="Sem sinal")
        self.record_btn.config(state="disabled")
        self.log("info", "Desconectado.")

    def get_esp_target(self):
        """Lido pela thread de tracking a cada envio UDP — permite mudar IP/porta em tempo real."""
        ip = self.ip_entry.get().strip() or DEFAULT_ESP32_IP
        try:
            port = int(self.port_entry.get().strip())
        except ValueError:
            port = DEFAULT_ESP32_PORT
        return ip, port

    def send_variables(self):
        """
        Monta o pacote 'CFG,kpx=...,kix=...,...' com as 6 variáveis de PID
        e envia via UDP para o ESP32 (mesmo IP/porta usados pelo tracking).
        O firmware reconhece esse pacote pelo prefixo 'CFG,' e atualiza as
        variáveis correspondentes sem afetar o watchdog do tracking.
        """
        pares = []
        for chave, rotulo, _padrao in PID_VARS:
            texto = self.pid_entries[chave].get().strip()
            try:
                valor = float(texto)
            except ValueError:
                self.log("erro", f"Valor inválido em {rotulo}: '{texto}' não é um número.")
                return
            pares.append(f"{chave}={valor}")

        payload = "CFG," + ",".join(pares)
        ip, port = self.get_esp_target()

        try:
            self._cfg_sock.sendto(payload.encode(), (ip, port))
        except OSError as e:
            self.log("erro", f"Falha ao enviar configuração via UDP: {e}")
            return

        self.log("info", f"Configuração PID enviada a {ip}:{port} -> {payload}")

    # ------------------- GRAVAÇÃO DE DADOS ------------------
    def toggle_recording(self):
        """Chamado pelo botão 'Iniciar/Parar Gravação'. Cria/fecha o CSV via FaceTrackerWorker."""
        if self.worker is None:
            return

        if not self.recording:
            path = self.worker.start_recording()
            if path is None:
                # start_recording já loga o erro no Monitor/Erros
                return
            self.recording = True
            self.record_btn.config(text="Parar Gravação", bg="#FF6B6B")
            self.record_status_label.config(
                text=f"Gravando em:\n{path}", fg="#a30000"
            )
        else:
            self.worker.stop_recording()
            self.recording = False
            self.record_btn.config(text="Iniciar Gravação", bg="#FFD700")
            self.record_status_label.config(text="Não gravando.", fg="#555555")
    # -----------------------------------------------------------------------------

    # -------------------- ATUALIZAÇÃO DA GUI (não bloqueante) --------------------
    def _schedule_gui_update(self):
        if not self.connected or self.worker is None:
            return

        frame = self.worker.get_display_frame()
        if frame is not None:
            self._update_camera_label(frame)

        with self.worker.stats_lock:
            stats = dict(self.worker.stats)

        self.stats_labels["Alvo"].config(text="Sim" if stats["alvo"] else "Não")
        self.stats_labels["ErroX"].config(text=str(stats["erroX"]))
        self.stats_labels["ErroY"].config(text=str(stats["erroY"]))
        self.stats_labels["FPS"].config(text=f"{stats['fps']:.1f}")

        # Reagenda — isso é o "loop" da GUI, leve e não bloqueante (não interfere no tracking)
        self.root.after(GUI_REFRESH_MS, self._schedule_gui_update)

    def _update_camera_label(self, frame_bgr):
        # Redimensiona para caber no painel sem distorcer
        h, w, _ = frame_bgr.shape
        scale = min(CAM_DISPLAY_W / w, CAM_DISPLAY_H / h)
        new_w, new_h = int(w * scale), int(h * scale)
        resized = cv2.resize(frame_bgr, (new_w, new_h))

        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(rgb)
        img_tk = ImageTk.PhotoImage(image=img_pil)

        self.cam_label.config(image=img_tk, text="")
        self.cam_label.image = img_tk  # mantém referência (evita garbage collection)

    # -------------------- FECHAMENTO --------------------
    def on_close(self):
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = FaceTrackerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()