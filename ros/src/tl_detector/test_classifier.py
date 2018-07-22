from light_classification.tl_classifier import TLClassifier
from PIL import Image
import os

lc = TLClassifier()

image = Image.open(os.path.dirname(os.path.abspath(__file__))+"/light_classification/tf1.jpg")

lc.get_classification(image)

