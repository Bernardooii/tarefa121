# importe o cv2 para capturar o feed de vídeo
import cv2

import numpy as np

# anexe a câmera indexada como 0
camera = cv2.VideoCapture(0)

# definindo a largura do quadro e a altura do quadro como 640 X 480
camera.set(3 , 640)
camera.set(4 , 480)

# carregando a imagem da montanha
mountain = cv2.imread('mount everest.jpg')

# redimensionando a imagem da montanha como 640 X 480
output_file = cv2.VideoWriter('output.avi', mountain, 20.0, (640, 480))   


while True:

    # ler um quadro da câmera conectada
    status , frame = camera.read()

    # se obtivermos o quadro com sucesso
    if status:

        # inverta-o
        frame = cv2.flip(frame , 1)

        # convertendo a imagem em RGB para facilitar o processamento
        frame_rgb = cv2.cvtColor(frame , cv2.COLOR_BGR2RGB)

        # criando os limites
        lower_bound = np.array([])
        upper_bound = np.array([])

        # imagem dentro do limite
        while (camera.isOpened()):
            ret, img = camera.read()
            if not ret:
                break


        # invertendo a máscara
        img = np.flip(img, axis=1)

        # bitwise_and - operação para extrair o primeiro plano / pessoa
        mask_1 = cv2.morphologyEx(mask_1,cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
        mask_1 = cv2.morphologyEx(mask_1,cv2.MORPH_DILATE, np.ones((3,3),np.uint8))

        # imagem final

        # exiba-a
        cv2.imshow('quadro' , frame)

        # espera de 1ms antes de exibir outro quadro
        code = cv2.waitKey(1)
        if code  ==  32:
            break
        mask_2 = cv2.bitwise_not(mask_1)
        res_1 = cv2.bitwise_and(img, img, mask = mask_2)
        res_2 = cv2.bitwise_and(bg, bg, mask = mask_1)

        final_output = cv2.addWeighted(res_1, 1, res_2, 1, 0)
    
        #Exibindo o resultado para o usuário
        output_file.write(final_output)
        cv2.imshow("magic", final_output)
    
    cv2.waitKey(1)
  
# libere a câmera e feche todas as janelas abertas
camera.release()
cv2.destroyAllWindows()
