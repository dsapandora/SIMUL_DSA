import cv
import numpy as np

def calc_hog(im,numorient=9):
    """
    calculate integral HOG (Histogram of Orientation Gradient) image (w,h,numorient)
    
    calc_hog(im, numorient=9)
    
    returns 
        Integral HOG image

    params 
        im : color image
        numorient : number of orientation bins, default is 9 (-4..4)
    
    """
    sz = cv.GetSize(im)
    gr = cv.CreateImage(sz, 8, 1)
    gx = cv.CreateImage(sz, 32, 1)
    gy = cv.CreateImage(sz, 32, 1)
    
    #convert to grayscale
    cv.CvtColor(im, gr, cv.CV_BGR2GRAY)
    
    #calc gradient using sobel
    cv.Sobel(gr, gx, 1, 0, 3)
    cv.Sobel(gr, gy, 0, 1, 3)
    
    #calc initial result
    hog = np.zeros((sz[1], sz[0], numorient))
    mid = numorient/2
    for y in xrange(0, sz[1]-1):
        for x in xrange(0, sz[0]-1):
            angle = int(round(mid*np.arctan2(gy[y,x], gx[y,x])/np.pi))+mid
            magnitude = np.sqrt(gx[y,x]*gx[y,x]+gy[y,x]*gy[y,x])
            hog[y,x,angle] += magnitude
            
            
    #build integral image
    for x in xrange(1, sz[0]-1):
        for ang in xrange(numorient):
            hog[y,x,ang] += hog[y,x-1,ang]
    for y in xrange(1, sz[1]-1):
        for ang in xrange(numorient):
            hog[y,x,ang] += hog[y-1,x,ang]
    for y in xrange(1, sz[1]-1):
        for x in xrange(1, sz[0]-1):
            for ang in xrange(numorient):
                #tambah kiri dan atas, kurangi dengan kiri-atas
                hog[y,x,ang] += hog[y-1,x,ang] + hog[y,x-1,ang] - hog[y-1,x-1,ang]
    return hog

def calc_hog_block(hogim, r):
    """
    calculate HOG feature given a rectangle and integral HOG image
    
    returns
        HOG feature (not normalized)

    params
        hogim : integral HOG image
        r : 4-tuple representing rect (left,top,right,bottom)
    """
    numorient = hogim.shape[2]
    result = np.zeros(numorient)
    for ang in xrange(numorient):
        result[ang] = hogim[r[1],r[0],ang] + hogim[r[3],r[2],ang] - hogim[r[1],r[2],ang] - hogim[r[3],r[0],ang]

    return result 

def draw_hog(target, ihog, cellsize=8):
    """
    visualize HOG features
    
    returns
        None
        
    params
        target  : target image
        ihog    : integral HOG image
        cellsize: size of HOG feature to be visualized (default 8x8)
        
    """
    ow,oh = cv.GetSize(target)
    halfcell = cellsize/2
    w,h = ow/cellsize,oh/cellsize
    norient = ihog.shape[2]
    mid = norient/2

    for y in xrange(h-1):
        for x in xrange(w-1):
            px,py=x*cellsize,y*cellsize
            #feat = calc_hog_block(ihog, (px,py,max(px+cellsize, ow-1),max(py+cellsize, oh-1)))
            feat = calc_hog_block(ihog, (px, py, px+cellsize, py+cellsize))
            px += halfcell
            py += halfcell
            
            #L1-norm, nice for visualization
            mag = np.sum(feat)
            maxv = np.max(feat)
            if mag > 1e-3:
                nfeat = feat/maxv
                N = norient
                fdraw = []
                for i in xrange(N):
                    angmax = nfeat.argmax()
                    valmax = nfeat[angmax]
                    x1 = int(round(valmax*halfcell*np.sin((angmax-mid)*np.pi/mid)))
                    y1 = int(round(valmax*halfcell*np.cos((angmax-mid)*np.pi/mid)))
                    gv = int(round(255*feat[angmax]/mag))
                    
                    #don't draw if less than a threshold
                    if gv < 30:
                        break
                    fdraw.insert(0, (x1,y1,gv))
                    nfeat[angmax] = 0.
                    
                #draw from smallest to highest gradient magnitude
                for i in xrange(len(fdraw)):
                    x1,y1,gv = fdraw[i]
                    cv.Line(target, (px-x1,py+y1), (px+x1,py-y1), cv.CV_RGB(gv, gv, gv), 1, 8)
            else:
                #don't draw if there's no reponse
                pass
im = cv.LoadImage('gente.jpg')

#image for visualization
vhog = cv.CreateImage(cv.GetSize(im), 8, 1)

hog = calc_hog(im)

draw_hog(vhog, hog, 8)
cv.ShowImage('hi', vhog)

#clear for reuse
cv.Set(vhog, 0)

draw_hog(vhog, hog, 16)
cv.ShowImage('lo', vhog)

key = cv.WaitKey(0)   
