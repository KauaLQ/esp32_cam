#include <Arduino.h>

extern volatile float KpX;
extern volatile float KiX;
extern volatile float KdX;

extern volatile float KpY;
extern volatile float KiY;
extern volatile float KdY;

String htmlPage() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<title>PID Controller</title>
<style>
body{
    font-family: Arial;
    margin:40px;
}
input{
    width:100px;
    margin:5px;
}
button{
    padding:10px;
}
</style>
</head>
<body>

<h2>Configuração PID</h2>

<form action="/update" method="GET">

<h3>Eixo X</h3>

KpX:
<input type="number" step="0.0001" name="kpx" value=")rawliteral" + String(KpX, 4) + R"rawliteral("><br>

KiX:
<input type="number" step="0.0001" name="kix" value=")rawliteral" + String(KiX, 4) + R"rawliteral("><br>

KdX:
<input type="number" step="0.0001" name="kdx" value=")rawliteral" + String(KdX, 4) + R"rawliteral("><br>

<h3>Eixo Y</h3>

KpY:
<input type="number" step="0.0001" name="kpy" value=")rawliteral" + String(KpY, 4) + R"rawliteral("><br>

KiY:
<input type="number" step="0.0001" name="kiy" value=")rawliteral" + String(KiY, 4) + R"rawliteral("><br>

KdY:
<input type="number" step="0.0001" name="kdy" value=")rawliteral" + String(KdY, 4) + R"rawliteral("><br><br>

<button type="submit">Salvar</button>

</form>

<hr>

<h3>Valores atuais</h3>

<p>KpX = )rawliteral" + String(KpX, 4) + R"rawliteral(</p>
<p>KiX = )rawliteral" + String(KiX, 4) + R"rawliteral(</p>
<p>KdX = )rawliteral" + String(KdX, 4) + R"rawliteral(</p>

<p>KpY = )rawliteral" + String(KpY, 4) + R"rawliteral(</p>
<p>KiY = )rawliteral" + String(KiY, 4) + R"rawliteral(</p>
<p>KdY = )rawliteral" + String(KdY, 4) + R"rawliteral(</p>

</body>
</html>
)rawliteral";

    return html;
}