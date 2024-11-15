from detectron2.utils.logger import setup_logger
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog


class ObjectDetector:
    def __init__(self, model_name='COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml'):
        setup_logger()
        self.cfg = get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file(model_name))
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
        self.cfg.MODEL.DEVICE = 'cpu'
        self.cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(model_name)
        self.predictor = DefaultPredictor(self.cfg)

    def process_image(self, image):
        outputs = self.predictor(image)
        metadata = MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0])
        visualizer = Visualizer(image[:, :, ::-1], metadata, scale=1.2)
        annotated_image = visualizer.draw_instance_predictions(outputs["instances"].to("cpu")).get_image()[:, :, ::-1]
        return annotated_image, outputs 


class LabelMapper:
    def __init__(self, label_file_path):
        self.class_names = self.load_labels(label_file_path)
    
    def load_labels(self, file_path):
        with open(file_path, 'r') as file:
            labels = file.read().splitlines()
        return labels

    def get_class_name(self, class_id):
        if 0 <= class_id < len(self.class_names):
            return self.class_names[class_id]
        return "Unknown"