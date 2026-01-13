import sensor, time, ml, uos, gc, pyb
from machine import LED

# ------------------------- Inicialização -------------------------

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.skip_frames(time=2000)
ledB = LED("LED_BLUE")
ledR = LED("LED_RED")
clock = time.clock()

# ------------------------ Calibration -----------------------------
#===================================================================

calibrate_mode = False

#antiga calibração

red_threshold = (22, 42, 29, 49, 9, 29)
yellow_threshold = (43, 63, -2, 18, 26, 46)
green_threshold = (33, 53, -37, -17, 5, 25)

take_photo_mode = False

NUM_FOTOS = 30
INTERVALO_MS = 1000

I2C_ADDR = 0x11  # Channel 0x11 Cam LEFT, Channel 0x13 Cam RIGHT

#====================================================================
# ------------------------- Comunicação I2C -------------------------

bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=I2C_ADDR)
bus.deinit()
bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=I2C_ADDR)

buffer = bytearray([0, 0])  # [valor identificado, confiabilidade]
print("Esperando o ESP32...")

def enviar_i2c():
    try:
        cmd = bus.recv(1, timeout=100)
        if cmd and cmd[0] == 0x00:
            bus.send(buffer)
            print(f"[I2C] Enviado: {buffer[0]}, {buffer[1]}")
            time.sleep_ms(30)
            ledB.on()
        else:
            print(f"[I2C] Comando inválido: {cmd}")
    except Exception as e:
        pass

# ------------------------- Processamento de Cor -------------------------

def calibrate_color(stats, margem=10):
    L_mean = stats.l_mean()
    A_mean = stats.a_mean()
    B_mean = stats.b_mean()

    L_min = max(0, int(L_mean - margem))
    L_max = min(100, int(L_mean + margem))
    A_min = max(-128, int(A_mean - margem))
    A_max = min(127, int(A_mean + margem))
    B_min = max(-128, int(B_mean - margem))
    B_max = min(127, int(B_mean + margem))

    return (L_min, L_max, A_min, A_max, B_min, B_max)

def show_calibrate(img):
    roi = (140, 100, 40, 40)
    img.draw_rectangle(roi, color=(255, 255, 255))
    stats = img.get_statistics(roi=roi)
    threshold = calibrate_color(stats)

    print("\n=========== Threshold =============")
    print(threshold)
    print("=====================================")

color_thresholds = [green_threshold, yellow_threshold, red_threshold]  # 1=verde, 2=amarelo, 3=vermelho

def detect_color(img):
    for i, threshold in enumerate(color_thresholds):
        blobs = img.find_blobs([threshold], pixels_threshold=500, area_threshold=1500, merge=True)
        for blob in blobs:
            w, h = blob.w(), blob.h()
            aspect_ratio = w / h if h != 0 else 0
            if 1 < aspect_ratio < 1.5:
                img.draw_rectangle(blob.rect(), color=(255, 0, 0))
                img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 0))
                return i + 1, 100  # Valor identificado e confiança alta
    return 0, 0

# ------------------------- Letters processing  -------------------------

try:
    net = ml.Model("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise Exception("Error loading model or labels: %s" % e)

# ------------------------------- Loop main ---------------------------------

while True:
    clock.tick()
    ledR.off()
    ledB.off()
    img = sensor.snapshot()
    img.rotation_corr(z_rotation=180)
    result = 0
    confidence = 0

    if calibrate_mode:
        show_calibrate(img)
        pyb.delay(1000)

    elif take_photo_mode:
        time.sleep_ms(1000)
        print("Capturando fotos...")

        for i in range(1, NUM_FOTOS + 1):
            img = sensor.snapshot()
            nome_arquivo = "foto_%d.jpg" % i
            img.save(nome_arquivo)
            print("Foto salva:", nome_arquivo)
            time.sleep_ms(INTERVALO_MS)

        print("Captura finalizada.")

    else:
        result, confidence = detect_color(img)
        if result == 0:
            predictions = net.predict([img])[0].flatten().tolist()
            max_index = predictions.index(max(predictions))
            predicted_label = labels[max_index]
            predicted_prob = int(predictions[max_index] * 100)

            if predicted_prob > 90:
                if predicted_label == "U":
                    result = 4
                elif predicted_label == "S":
                    result = 5
                elif predicted_label == "H":
                    result = 6
                confidence = predicted_prob

                #img.draw_string(0, 0, "Victim: %s" % predicted_label, mono_space=True)

        #print("Result:", result, "Confidence:", confidence)
        #print("FPS:", clock.fps())

        if result != 0 and confidence > 20:
            buffer[0] = result
            buffer[1] = confidence
            ledR.on()
        else:
            buffer[0] = 0
            buffer[1] = 0

        enviar_i2c()
        gc.collect()
