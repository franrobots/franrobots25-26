import serial
bluetooth_port = 'COM4'

try:
    bluetooth = serial.Serial(bluetooth_port, 9600)  
    print("Conexão Bluetooth estabelecida com sucesso.")
except serial.SerialException:
    print("Erro ao conectar ao dispositivo Bluetooth. Verifique se o dispositivo está emparelhado e conectado.")
    exit()

try:
    while True:
        data = bluetooth.readline().decode().strip()
        print(data)
except KeyboardInterrupt:
    print("Programa encerrado pelo usuário.")
    
bluetooth.close()
