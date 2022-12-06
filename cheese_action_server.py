import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import os
import cv2 as cv
from action_cheese.action import Cheese
import sys


class CheeseActionServer(Node):

    def __init__(self):
        super().__init__('cheese_action_server')
        self._action_server = ActionServer(
            self,
            Cheese,
            'cheese',
            self.execute_callback)
        self.camara = int(sys.argv[1])
        self.profile_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_profileface.xml')
        self.face_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_frontalface_default.xml')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        cap = cv.VideoCapture(self.camara)
        feedback_message = Cheese.Feedback()
        result = Cheese.Result()
        result.result = "NO"

        all_photos_taken = False
        photos_taken = 0

        while all_photos_taken == False:

            ret, frame = cap.read()

            if ret:

                faces = self.face_cascade.detectMultiScale(frame, 1.1, 4)
                profiles = self.profile_cascade.detectMultiScale(frame, 1.1, 4)
                faces = sorted(faces, key=lambda x: x[2]*x[3], reverse=True)
                profiles = sorted(profiles, key=lambda x: x[2]*x[3], reverse=True)
                faces = faces[:1]
                profiles = profiles[:1]

                if len(faces) == 1:

                    face = faces[0]
                    x_1, y_1, w_1, h_1 = face
                    cv.rectangle(frame, (x_1, y_1), (x_1+w_1, y_1+h_1), (0, 255, 0), 2)
                    if cv.waitKey(1) & 0xFF == ord('c'):
                        name = 'photo_' + str(photos_taken + 1)
                        photos_taken = photos_taken + \
                            self.take_photo(frame, x_1, y_1, w_1, h_1, name)
                        feedback_message.progress = "Fotos tomadas: " + \
                            str(photos_taken)
                        self.get_logger().info('Feedback: {0}'.format(
                            feedback_message.progress))
                        goal_handle.publish_feedback(feedback_message)

                elif len(profiles) == 1:
                    profile = profiles[0]
                    x_2, y_2, w_2, h_2 = profile
                    cv.rectangle(frame, (x_2, y_2), (x_2+w_2, y_2+h_2), (0, 255, 0), 2)
                    if cv.waitKey(1) & 0xFF == ord('c'):
                        name = 'photo_' + str(photos_taken + 1)
                        photos_taken = photos_taken + \
                            self.take_photo(frame, x_2, y_2, w_2, h_2, name)
                        feedback_message.progress = "Fotos tomadas: " + \
                            str(photos_taken)
                        self.get_logger().info('Feedback: {0}'.format(
                            feedback_message.progress))
                        goal_handle.publish_feedback(feedback_message)

                if photos_taken >= 5:
                    all_photos_taken = True
                    feedback_message.progress = "Todas las fotos tomadas, cerrando..."
                    result.result = "YES"
                    self.get_logger().info('Feedback: {0}'.format(
                        feedback_message.progress))
                    goal_handle.publish_feedback(feedback_message)

                cv.imshow("camera", frame)
                cv.waitKey(1)

        cv.destroyAllWindows()
        goal_handle.succeed()
        return result
        
    def take_photo(self, frame, x, y, w, h, name):
        save = frame[y:y+h, x:x+w]
        path = os.path.join(os.getcwd(), 'Resources', 'Persona', name + '.png') #Se tiene que ejecutar desde src
        status = cv.imwrite(path, save)
        if status == False:
            return 0
        else:
            return 1


def main(args=None):
    rclpy.init(args=args)

    cheese_action_server = CheeseActionServer()

    rclpy.spin(cheese_action_server)


if __name__ == '__main__':
    main()
