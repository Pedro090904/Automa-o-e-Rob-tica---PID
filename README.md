Robô de Duas Rodas com Autoequilíbrio (Pêndulo Invertido)

Este repositório contém o código-fonte, os esquemáticos elétricos e a documentação do projeto de um Robô Autobalanceador desenvolvido para a disciplina de Automação e Robótica (Centro Universitário IESB).

O sistema baseia-se no clássico problema de controlo do Pêndulo Invertido, utilizando um controlador PID em malha fechada processado num Arduino Uno.


🛠️ Tecnologias e Hardware Utilizado

Microcontrolador: Arduino Uno R3 (ATmega328P)

Sensor Inercial: MPU6050 (Acelerómetro + Giroscópio 6 eixos) utilizando o DMP interno para cálculo de Quaternions.

Driver de Potência: Módulo L298N (Configurado em 6 pinos para separação de direção e PWM).

Atuadores: Motores DC TT (6V) com caixa de redução.

Ambiente de Desenvolvimento: C++ via PlatformIO.

Simulação: Webots.

⚙️ Arquitetura de Software (Firmware)

O código foi desenhado com base numa arquitetura Não-Bloqueante (Event-Driven). Em vez de sobrecarregar o processador com pesquisas contínuas (polling), o Arduino aguarda interrupções de hardware (Pino 2) geradas pelo MPU6050. Isto garante uma frequência de amostragem constante (loop de ~100Hz) necessária para o rigor do cálculo PID.

A malha de controlo utiliza a técnica de "Derivada na Medição" para evitar solavancos (Derivative Kick) e limites de saturação de saída para prevenir o Integral Windup. Foi implementada também uma lógica Fail-Safe para cortar a potência dos motores caso o ângulo exceda a janela de recuperação (queda iminente).

📊 Estado Atual do Projeto e Limitações

A versão atual do projeto (Ponto de Controlo 4) possui a arquitetura teórica validada e o firmware totalmente implementado. Contudo, a integração física revelou limitações eletromecânicas que impediram o estado perfeito de autoequilíbrio:

Limitação de Corrente (Brownout): Devido à indisponibilidade de células de alta descarga (Li-Ion 18650), o sistema foi temporariamente testado com baterias alcalinas (9V). O pico de corrente exigido pelo PWM gerou afundamentos drásticos de tensão ($\sim 4.20V$), impedindo os motores de gerar o binário de recuperação necessário e causando o reinício (reset) do microcontrolador.

Falhas no Barramento I2C: Problemas de manufatura (soldadura fria) no módulo MPU6050 geraram instabilidade na comunicação de dados.

Trabalhos Futuros: Sugere-se a transição definitiva para packs de baterias LiPo/Li-Ion ($>2A$), uso de motores de passo (ou DC com encoder) para mitigar a folga mecânica (backlash) e a adoção de Protoshields para garantir a integridade elétrica das ligações I2C.

👥 Equipa de Desenvolvimento

Artur Rodrigues de Lima

Dara Yuna Borges Fujii

Maria Gabriella Requette Marinho

Mayra Sales da Costa

Pedro de Sousa Mesquita