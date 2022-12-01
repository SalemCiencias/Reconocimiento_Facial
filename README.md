# Reconocimiento_Facial
Modulo para reconocer a la persona a guiar.

- Puede que se necesite cambiar la siguiente linea en los archivos _cheese_action_server.py_ (línea 24) y _face_recognition_pub.py_ (línea 17) si cv no reconoce la cámara

    self.cap = cv.VideoCapture(2)
    
  Cambiar el 2 por el índice de la cámara a utilizar (checar /dev), la cámara por default es la 0

# cheese_action_server.py

- En una terminal correr
        ```python3 cheese_action_server.py```
- (Como alternativa para el cliente) En otra terminal correr
        ```ros2 action send_goal cheese action_cheese/action/Cheese "{order: inicia}"```
    - Tomará un momento en iniciar la cámara, oprimir _c_ en el teclado para tomar la foto

# cheese_action_client.py

- Después de hacer lo de arriba correr en otra terminal
        ```python3 cheese_action_client.py```
- Tomará un momento en iniciar la cámara, oprimir _c_ en el teclado para tomar la foto

# face_recognition_pub

- En una terminal correr 
        ```ros2 run face_recognition_pub talker```
        
- Es posible cambiar la línea

    if m.distance < X*n.distance:
    
  Se recomienda establecer X entre 70 y 80
