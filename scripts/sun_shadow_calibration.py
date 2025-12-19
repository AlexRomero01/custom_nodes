#!/usr/bin/env python3
import cv2
import numpy as np
import os

def main():
   
    img_path = os.path.expanduser("~/Pictures/Screenshots/aaa.png")
    # Leer imagen
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    hist= cv2.calcHist([img], [0], None, [256], [0, 256])
    hist = hist.ravel() / hist.sum() 
    print("Histograma normalizado:")
    print(hist)  

    if img is None:
        print("Error: no se pudo abrir la imagen.")
        return

    print("Matriz de intensidades (0–255):")
    print(img)  

    cv2.imshow("Grayscale", img)


if __name__ == "__main__":
    main()
