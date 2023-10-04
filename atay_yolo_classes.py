"""yolov3_classes.py

NOTE: Number of YOLOv3 COCO output classes differs from SSD COCO models.
"""

ATAY_CLASSES_LIST = [
    "Kirmizi_Isik",
    "Sari_Isik",
    "Yesil_Isik",
    "Dur",
    "Girilmez",
    "Sag",
    "Sag_Yasak",
    "Sag_Duz",
    "Sol",
    "Sol_Yasak",
    "Sol_Duz",
    "Hiz_30",
    "Hiz_20",
    "Durak",
    "Park",
    "Park_Yasak",
    "Kapali_Yol", 
]

def get_cls_dict(category_num=17):
    """Get the class ID to name translation dictionary."""
    return {i: n for i, n in enumerate(ATAY_CLASSES_LIST)}
