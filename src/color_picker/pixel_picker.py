import cv2
import numpy as np

shrinkHeight = 1.0
dotRadius = 5

def PointPickerTarCur(img, height_scale = shrinkHeight, r=dotRadius):

    global points, bluDot, redDot, greDot, res, rgba
    points = [[-1, -1], [-1, -1], [-1, -1]]
    
    # First create the image with alpha channel
    rgba = cv2.cvtColor(img, cv2.COLOR_RGB2RGBA)

    # Then assign the mask to the last channel of the image
    
    alphaChannel = np.full((img.shape[0], img.shape[1]), 128)
    rgba[:, :, 3] = alphaChannel
    #img = np.dstack((img, alphaChannel))
    
    bluDot = np.zeros((img.shape[0], img.shape[1], 4), dtype=np.uint8)
    redDot = np.zeros((img.shape[0], img.shape[1], 4), dtype=np.uint8)  
    greDot = np.zeros((img.shape[0], img.shape[1], 4), dtype=np.uint8)  
    
    def mouse_click1(event, x, y, 
                flags, param):
        
        actualX = int(x/height_scale)
        actualY = int(y/height_scale)

        global points, redDot, res, rgba
        if event == cv2.EVENT_LBUTTONDOWN:

            points[0] = [actualX,actualY]
            
            redDot = np.zeros((img.shape[0], img.shape[1], 4), dtype=np.uint8)  
            cv2.line(redDot, (actualX, 0), (actualX, redDot.shape[0]), (0,0,255,128), 2)
            cv2.line(redDot, (0, actualY), (redDot.shape[1], actualY), (0,0,255,128), 2)
        
        res = rgba.copy()
        cnd = redDot[:, :, 3] > 0
        res[cnd] = redDot[cnd]
        resshow = ResizeWithAspectRatio(res, height=int(res.shape[0]*height_scale))

        cv2.imshow('image', resshow)
        
    def mouse_click2(event, x, y, 
                flags, param):
        
        actualX = int(x/height_scale)
        actualY = int(y/height_scale)

        global points, bluDot, res, rgba
        if event == cv2.EVENT_LBUTTONDOWN:
            
            points[1] = [actualX,actualY]
            bluDot = np.zeros((img.shape[0], img.shape[1], 4), dtype=np.uint8)
            cv2.line(bluDot, (actualX, 0), (actualX, bluDot.shape[0]), (255,0,0,128), 2)
            cv2.line(bluDot, (0, actualY), (bluDot.shape[1], actualY), (255,0,0,128), 2)
        
        res = rgba.copy()
        cnd = bluDot[:, :, 3] > 0
        res[cnd] = bluDot[cnd]
        resshow = ResizeWithAspectRatio(res, height=int(res.shape[0]*height_scale))

        cv2.imshow('image', resshow)
    
    def mouse_click3(event, x, y, 
                flags, param):
        
        actualX = int(x/height_scale)
        actualY = int(y/height_scale)

        global points, greDot, res, rgba
        if event == cv2.EVENT_LBUTTONDOWN:
            
            points[2] = [actualX,actualY]
            greDot = np.zeros((img.shape[0], img.shape[1], 4), dtype=np.uint8)
            cv2.line(greDot, (actualX, 0), (actualX, greDot.shape[0]), (0,255,0,128), 2)
            cv2.line(greDot, (0, actualY), (greDot.shape[1], actualY), (0,255,0,128), 2)

        res = rgba.copy()
        cnd = greDot[:, :, 3] > 0
        res[cnd] = greDot[cnd]
        resshow = ResizeWithAspectRatio(res, height=int(res.shape[0]*height_scale))

        cv2.imshow('image', resshow)
        
    
    res = rgba.copy()
    resshow = ResizeWithAspectRatio(res, height=int(res.shape[0]*height_scale))
    cv2.imshow('image', resshow)   
    cv2.setMouseCallback('image', mouse_click1)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    res = rgba.copy()
    resshow = ResizeWithAspectRatio(res, height=int(res.shape[0]*height_scale))
    cv2.imshow('image', resshow)   
    cv2.setMouseCallback('image', mouse_click2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    res = rgba.copy()
    resshow = ResizeWithAspectRatio(res, height=int(res.shape[0]*height_scale))
    cv2.imshow('image', resshow)   
    cv2.setMouseCallback('image', mouse_click3)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return points

def PointPickerOne(img, height_scale = shrinkHeight, r=dotRadius):

    global points, bluDot, redDot, greDot, res, rgba
    points = [-1, -1]
    
    # First create the image with alpha channel
    rgba = cv2.cvtColor(img, cv2.COLOR_RGB2RGBA)

    # Then assign the mask to the last channel of the image
    
    alphaChannel = np.full((img.shape[0], img.shape[1]), 128)
    rgba[:, :, 3] = alphaChannel
    #img = np.dstack((img, alphaChannel))
    
    redDot = np.zeros((img.shape[0], img.shape[1], 4), dtype=np.uint8)  
    
    def mouse_click(event, x, y, 
                flags, param):
        
        actualX = int(x/height_scale)
        actualY = int(y/height_scale)

        global points, redDot, res, rgba
        if event == cv2.EVENT_LBUTTONDOWN:

            points = [actualX,actualY]
            
            redDot = np.zeros((img.shape[0], img.shape[1], 4), dtype=np.uint8)  
            cv2.line(redDot, (actualX, 0), (actualX, redDot.shape[0]), (0,0,255,128), 2)
            cv2.line(redDot, (0, actualY), (redDot.shape[1], actualY), (0,0,255,128), 2)
        
        res = rgba.copy()
        cnd = redDot[:, :, 3] > 0
        res[cnd] = redDot[cnd]
        resshow = ResizeWithAspectRatio(res, height=int(res.shape[0]*height_scale))

        cv2.imshow('image', resshow)     
    
    res = rgba.copy()
    resshow = ResizeWithAspectRatio(res, height=int(res.shape[0]*height_scale))
    cv2.imshow('image', resshow)   
    cv2.setMouseCallback('image', mouse_click)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return points

def ResizeWithAspectRatio(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    return cv2.resize(image, dim, interpolation=inter)
