
# Detecção de Movimento da Mão para Controle de Relé via OpenCV e Arduino

Este projeto implementa um sistema de detecção de movimento da mão usando OpenCV para controlar um relé conectado ao Arduino. A câmera detecta quando a mão está posicionada sobre um ponto específico da tela e envia um comando ao Arduino para ativar ou desativar o relé, permitindo o controle de dispositivos externos.

## Funcionalidades

- **Detecção de Movimento**: Usa OpenCV para detectar a posição da mão em relação a um ponto fixo na tela.
- **Controle de Relé**: Envia comandos para o Arduino via comunicação serial para ativar/desativar um relé.
- **Configurações Personalizáveis**: Fácil ajuste de parâmetros, como o ponto fixo e o raio de proximidade.

## Tecnologias Utilizadas

- **Python 3.x**
- **OpenCV** - para captura de vídeo e processamento de imagem.
- **NumPy** - para cálculos de distância.
- **PySerial** - para comunicação com o Arduino.

## Configuração

1. Conecte o Arduino ao computador e identifique a porta serial correspondente. Substitua `'COM5'` no código pela porta adequada:
   ```python
   arduino = serial.Serial('COM5', 9600)
   ```
2. Instale as dependências do projeto:
   ```bash
   pip install opencv-python-headless numpy pyserial
   ```

## Uso

1. Conecte o relé ao Arduino e ajuste o código para corresponder à configuração do seu circuito.
2. Execute o script Python:
   ```bash
   python main.py
   ```
3. Uma janela de captura de vídeo será exibida, com um ponto verde indicando o ponto fixo.
4. Quando a mão for detectada sobre o ponto fixo, o relé será ativado. Ao sair do ponto, o relé será desligado.

### Interação com o Código

- Pressione `q` para fechar a janela e encerrar o programa.

## Código

As principais funções do projeto incluem:

- **detect_hand_movement(frame)**: Detecta a posição da mão com base em um intervalo de cor definido para a pele.
- **Controle do Relé**: O relé é acionado quando a posição da mão está dentro do raio definido ao redor do ponto fixo.

## Contribuição

Se desejar contribuir com melhorias ou novas funcionalidades, sinta-se à vontade para abrir um pull request.

## Licença

Este projeto está licenciado sob a [MIT License](LICENSE).
