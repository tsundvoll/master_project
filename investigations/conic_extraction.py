import matplotlib.pyplot as plt
import cv2
import numpy as np
import random
from math import pi


def load_ellipse_from_file():
    filepath = 'ellipse.jpg'

    img = cv2.imread(filepath) # import as BGR
    winname = "image"
    cv2.namedWindow(winname)        # Create a named window
    cv2.moveWindow(winname, 40,500)  # Move it to (40,30)
    cv2.imshow(winname, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


    print(type(img))

    ellipse = []

    for row in range(255):
        for col in range(255):
            if not np.array_equal(img[row][col], [255, 255, 255]):
                # print(img[row][col])
                ellipse.append([row, col])

    np_ellipse = np.array(ellipse)

    len_ellipse = np_ellipse.shape[0]
    print(len_ellipse)


    # id_sample = np.random.choice(len_ellipse, 10, replace=False)

    # Choose the same points every time for easier comparison
    id_sample = range(0,len_ellipse, 20)
    print(id_sample)


    np_ellipse_sample = np_ellipse[id_sample]

    print(np_ellipse_sample)

    plt.figure(figsize=(8,8))
    plt.plot(np_ellipse_sample[:,1], 255-np_ellipse_sample[:,0], 'go')
    plt.xlim((0,255))
    plt.ylim((0,255))
    plt.show()

scattered_ellipse = np.array(
    [   [ 66, 96],
        [ 70, 69],
        [ 74, 133],
        [ 82, 43],
        [ 92, 152],
        [109, 157],
        [130, 37],
        [142, 133],
        [148, 118],
        [151, 105]
    ]
)

scattered_ellipse2 = np.array(
    [   [ 50, 50],
        [ 30, 70],
        [ 70, 70],
        [150, 150],
        [130, 37],
        [142, 133]
    ]
)

# plt.figure(figsize=(8,8))
# plt.plot(scattered_ellipse[:,1], 255-scattered_ellipse[:,0], 'go')
# plt.xlim((0,255))
# plt.ylim((0,255))
# plt.show()

def fit_ellipse(points):
    x = points[:,1]
    y = 255-points[:,0]

    print(x.shape)

    print('x:', x)
    print('y:', y)

    # plt.figure(figsize=(8,8))
    # plt.plot(x, y, 'go')
    # plt.xlim((0,255))
    # plt.ylim((0,255))
    # plt.show()

    D11 = np.square(x)
    D12 = x*y
    D13 = np.square(y)
    D1 = np.array([D11, D12, D13]).T
    print("D1:")
    print(D1)
    print

    D2 = np.array([x, y, np.ones(x.shape[0])]).T
    print("D2:")
    print(D2)
    print

    S1 = np.dot(D1.T,D1)
    print("S1:")
    print(S1)
    print

    S2 = np.dot(D1.T,D2)
    print("S2:")
    print(S2)
    print

    S3 = np.dot(D2.T,D2)
    print("S3:")
    print(S3)
    print

    inv_S3 = np.linalg.inv(S3)
    print("inv_S3:")
    print(inv_S3)
    print

    T = - np.dot(inv_S3, S2.T) # for getting a2 from a1
    print("T:")
    print(T)
    print

    M = S1 + np.dot(S2, T)

    C1 = np.array([
        [0, 0, 0.5],
        [0, -1, 0],
        [0.5, 0, 0]
    ])

    M = np.dot(C1, M) # This premultiplication can possibly be made more efficient
    print("M:")
    print(M)
    print

    eigenvalues, eigenvectors = np.linalg.eig(M)
    print("eigenvalues:")
    print(eigenvalues)
    print
    print("eigenvectors:")
    print(eigenvectors)
    print

    cond = 4*eigenvectors[0]*eigenvectors[2] - np.square(eigenvectors[0])
    print("cond:")
    print(cond)
    print

    a1 = eigenvectors[:,cond > 0]
    print("a1:")
    print(a1)
    print

    a = np.concatenate((a1, np.dot(T, a1)))
    print("a:")
    print(a)
    print


    delta = 0.025
    xrange = np.arange(0, 255, delta)
    yrange = np.arange(0, 255, delta)
    X, Y = np.meshgrid(xrange,yrange)

    # F is one side of the equation, G is the other
    F = a[0]*X*X + a[1]*X*Y + a[2]*Y*Y + a[3]*X + a[4]*Y + a[5]
    G = 0


    plt.figure(figsize=(8,8))
    plt.plot(x, y, 'go')
    plt.xlim((0,255))
    plt.ylim((0,255))
    plt.contour(X, Y, (F - G), [0])
    plt.show()


    # u=1.     #x-position of the center
    # v=0.5    #y-position of the center
    # a=2.     #radius on the x-axis
    # b=1.5    #radius on the y-axis

    # t = np.linspace(0, 2*pi, 100)
    # plt.figure(figsize=(8,8))
    # plt.plot( u+a*np.cos(t) , v+b*np.sin(t) )
    # plt.xlim((0,255))
    # plt.ylim((0,255))
    # plt.grid(color='lightgray',linestyle='--')
    # plt.show()




# fit_ellipse(scattered_ellipse)
# a:
# [[-3.66860513e-01]
#  [-3.88682954e-03]
#  [-9.30267841e-01]
#  [ 6.50013809e+01]
#  [ 2.03228556e+02]
#  [-1.21953571e+04]]

def make_blurry(image, blurr):
    return cv2.medianBlur(image, blurr)


def hsv_make_orange_to_green(hsv):
    bgr_green = np.uint8([[[30,90,30]]])
    hsv_green = cv2.cvtColor(bgr_green,cv2.COLOR_BGR2HSV)

    lower_orange = np.array([8,128,64])
    upper_orange = np.array([28,255,255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # change the orange to green
    imask = mask>0
    orange_to_green = hsv.copy()
    orange_to_green[imask] = hsv_green

    return orange_to_green


def hsv_find_green_mask(hsv):
    bgr_green = np.uint8([[[30,90,30]]])
    hsv_green = cv2.cvtColor(bgr_green,cv2.COLOR_BGR2HSV)

    lower_green_h = 50
    lower_green_s = 50 * 0.01*255
    lower_green_v = 25 * 0.01*255

    upper_green_h = 70
    upper_green_s = 100 * 0.01*255
    upper_green_v = 70 * 0.01*255

    lower_green = np.array([lower_green_h,lower_green_s,lower_green_v])
    upper_green = np.array([upper_green_h,upper_green_s,upper_green_v])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
  
    # keep only the green
    imask = green_mask>0
    green = np.zeros_like(hsv, np.uint8)
    green[imask] = hsv_green
  
    return green


def hsv_make_grayscale(image):
    bgr = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return gray


def hsv_save_image(image, label='image', is_gray=False):
    folder = 'image_processing/'
    if is_gray:
        cv2.imwrite(folder+label+".png", image)
    else:
        cv2.imwrite(folder+label+".png", cv2.cvtColor(image, cv2.COLOR_HSV2BGR))

    return image


def load_image():
    filepath = 'image.jpg'

    img = cv2.imread(filepath) # import as BGR

    # winname = "image"
    # cv2.namedWindow(winname)        # Create a named window
    # cv2.moveWindow(winname, 40,500)  # Move it to (40,30)
    # cv2.imshow(winname, img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = hsv_save_image(hsv, '1_hsv')
    # ### Preprocessing ###
    hsv = hsv_save_image(hsv_make_orange_to_green(hsv), '2_orange to green')
    hsv = make_blurry(hsv, 3)
    hsv = make_blurry(hsv, 3)
    hsv = make_blurry(hsv, 3)
    hsv = hsv_save_image(make_blurry(hsv, 3), '3_make blurry')
    hsv = hsv_save_image(hsv_find_green_mask(hsv), '4_green mask')
    hsv = hsv_save_image(make_blurry(hsv, 9), '5_make blurry_2')
    gray = hsv_save_image(hsv_make_grayscale(hsv), '6_make grayscale', is_gray=True)
    gray = make_blurry(gray, 31)
    gray = hsv_save_image(make_blurry(gray, 21), '7_make blurry_3', is_gray=True)





load_image()