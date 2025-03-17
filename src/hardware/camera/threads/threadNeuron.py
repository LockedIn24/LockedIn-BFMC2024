
from src.templates.threadwithstop import ThreadWithStop
from ultralytics import YOLO

class threadNeuron(ThreadWithStop):
    
    def __init__(self, queueImage, queueSign):
        self.queueImage = queueImage
        self.queueSign = queueSign 
        
        
        self.model = YOLO("/home/oncst/Weights/weights/best.pt")
        
    def subscribe(self):
        pass
        
    def run(self):
        
        while self._running:
            img = self.queueImage.get()
            
            if img is not None:
                cropped_image = img[0:205, 256:512]
                results = self.model(cropped_image, verbose = False)
                max_size = -0.6
                classId = -1                    
                if results is not None and hasattr(results[0],"boxes"):
                    for box in results[0].boxes:  # Each detection
                        x_min, y_min, x_max, y_max = box.xyxy.tolist()[0]
                        bbox_width = x_max - x_min
                        bbox_height = y_max - y_min
                        objectId = int(box.cls.item())
                        surface_area = bbox_width * bbox_height
                        if surface_area > max_size:
                            classId = objectId
                            max_size = surface_area
                
                    
                if max_size > 1500:
                    self.queueSign.put(classId)
