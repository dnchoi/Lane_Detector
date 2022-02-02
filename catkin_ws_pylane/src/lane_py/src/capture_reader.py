import cv2
import sys
class video_cap:
    def __init__(self, cam_video, res, fps):
        super().__init__()
        self.cap = cv2.VideoCapture(cam_video)
        self.STD_DIMENSIONS =  {
            "240p": (320, 240),
            "480p": (640, 480),
            "720p": (1280, 720),
            "1080p": (1920, 1080),
            "4k": (3840, 2160),
        }
        self.get_dims(self.cap, fps, res)
    
    def change_res(self, cap, width, height, fps):
        self.cap.set(3, width)
        self.cap.set(4, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
    def get_dims(self, cap, frame_rate, res='480p'):
        width, height = self.STD_DIMENSIONS["480p"]
        if res in self.STD_DIMENSIONS:
            width,height = self.STD_DIMENSIONS[res]
        self.change_res(self.cap, width, height, frame_rate)
        return width, height
    
    def is_open(self):
        return self.cap.isOpened()

    def check_cam_width_height(self):
        print(self.cap.get(3), self.cap.get(4))

        return self.cap.get(3), self.cap.get(4)
    
    def _read_image(self):
        ret, frame = self.cap.read()
        return frame if ret else None

    def read_frame(self):
        frame = self._read_image()
        if frame is not None:
            if frame.shape[1] > 640:
                frame = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_AREA)
            return frame
        else:
            print("it is video or camera exit")
            sys.exit()
    
    def close_cap(self):
        self.cap.release()
        cv2.destroyAllWindows()