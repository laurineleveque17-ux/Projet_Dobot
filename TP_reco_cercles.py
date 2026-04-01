import numpy
import threading
import DobotDllType as dType
import cv2

# Dobot_commande ______________________________________________________________
# Il s'agit de programmer la mise en mouvement en ligne droite de la pointe du Feutre vers les centres des cercles
# détectés par la caméra sachant que le bras du robot devra rejoindre le point de coordonnée (x=250, y=0, z=50 mm)
# à l’issue de chaque descente vers le centre d’un cercle. Pour exprimer dans le repère de base du robot (R_0)
# des coordonnées exprimées dans le repère Feuille, on doit fixer :
#  - la valeur du vecteur O_0 O_F dans le repère de base du robot (R_0) (utiliser pour cela l'appli DobotStudio),
#  - la valeur de l'angle en rd entre X_0 et X_Feuille

# choix de la source de la caméra (0 pour une webcam intégrée à l'ordinateur, 1 pour une webcam externe sur port USB)
cap = cv2.VideoCapture(1,cv2.CAP_DSHOW)
# initialisation de la résolution (axe X = 1920, axe Y = 1080 px)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

ret, frame = cap.read()  # retourne une image dans la variable frame

mire_px = 300
mire_mm = 21.5*2

px_en_mm=mire_mm/mire_px  # calcul de la taille d'un pixel en mm (mire_mm et mire_px sont définis dans le fichier Dobot_calibration_mire.py)

# --- CRÉATION DU MASQUE A4 ---
# On calcule la taille de la feuille A4 en pixels selon VOTRE calibration
# Largeur feuille (mm) / taille d'un pixel (mm) = Largeur en pixels
width_a4_px = int(297 / px_en_mm)
height_a4_px = int(210 / px_en_mm)

# On récupère le centre de l'image caméra
h_img, w_img = frame.shape[:2] # h=1080, w=1920
cent_x, cent_y = w_img // 2, h_img // 2

# On définit les coins du rectangle A4 autour du centre de l'image
x1 = int(cent_x - (width_a4_px / 2))
x2 = int(cent_x + (width_a4_px / 2))
y1 = int(cent_y - (height_a4_px / 2))
y2 = int(cent_y + (height_a4_px / 2))

# Sécurité : on s'assure de ne pas dépasser les bords de l'image
x1, x2 = max(0, x1), min(w_img, x2)
y1, y2 = max(0, y1), min(h_img, y2)

mask = numpy.zeros(frame.shape[:2], numpy.uint8)
mask[y1:y2, x1:x2] = 255 # On dessine le rectangle blanc correspondant au A4
frame = cv2.bitwise_and(frame,frame,mask = mask)

cv2.imwrite('image_initiale_avec_masque_A4.png', frame)  # enregistre l'image 'frame' dans le fichier 'image_intiale_avec_masque_A4.png'
cap.release()  # ferme le flux caméra pour libérer des ressources
# passage de l'image enregistrée en niveaux de gris
img = cv2.imread('image_initiale_avec_masque_A4.png', cv2.IMREAD_GRAYSCALE)

seuil = 100  # valeur du seuil utilisée lors de la binarisation de l'image 'img' (chaque pixel est mis : à 0
# (noir) si sa valeur est < au seuil, à 255 (blanc)) sinon)
ret1, gray = cv2.threshold(img, seuil, 255, cv2.THRESH_BINARY)
cv2.imshow('image binaire', gray)  # affiche l'image 'gray' dans la fenêtre 'image binarisée'
cv2.waitKey(0)  # attend appui sur une touche pour ne plus afficher la fenêtre

# utilisation de la transformée de Hough (HoughCircles) pour détecter les cercles dans l'image 'gray'
rows = gray.shape[0]  # rows contient le nombre de ligne contenue dans l'image
circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8, param1=100, param2=30, minRadius=1, maxRadius=300)

if circles is not None:
    print(circles.shape[1], 'cercle(s) détecté(s) :-)')
    center_mm = numpy.zeros((circles.shape[1], 2))
    j = 0
    circles = numpy.uint16(numpy.around(circles))
    for i in circles[0, :]:
        center_px = (i[0], i[1])
        print('centre X, Y (px) = ', center_px)
        # calcul de la variable center_mm[j, :] en fonction des variables i[0], i[1]
        center_mm[j, :] = [i[0]*px_en_mm, i[1]*px_en_mm]
        print('centre X, Y (mm) = ', center_mm[j, 0], center_mm[j, 1])
        j = j+1
        # circle center
        cv2.circle(frame, center_px, 3, (0, 0, 255), 3)  # rouge (système BGR)
        # circle outline
        radius = i[2]
        cv2.circle(frame, center_px, radius, (0, 255, 0), 3)  # vert
    cv2.imshow('cercle avec centre', frame)  # affiche l'image 'img' dans la fenêtre 'cercle avec centre'
    cv2.waitKey(0)  # attend appui sur une touche pour ne plus afficher la fenêtre
    cv2.imwrite('cercle_avec_centre.png', frame)  # sauvegarde de l'image 'img' dans le fichier 'cercle_avec_centre.png'
else:
    print('aucun cercle détecté !!')

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

#Load Dll
api = dType.load()

#Connect Dobot
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:",CON_STR[state])

if (state == dType.DobotConnect.DobotConnect_NoError):

    # 1. Arrêter immédiatement tout mouvement résiduel
    dType.SetQueuedCmdForceStopExec(api)
    
    # 2. Effacer les alarmes (lumière rouge -> verte)
    # C'est ce qui permet de débloquer le robot après une butée atteinte
    dType.ClearAllAlarmsState(api)

    #Clean Command Queued
    dType.SetQueuedCmdClear(api)

    #Async Motion Params Setting
    dType.SetHOMEParams(api, 250, 0, 150, 0, isQueued = 1)
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
    dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)

    #Setting the offset of the end-effector
    # outil correspondant à la pointe du Feutre (X_bias=61, Y_bias=0, Z_bias= 82.7 mm)
    dType.SetEndEffectorParams(api, 61, 0, 82.7, isQueued=1)

    #Commande Home
    dType.SetHOMECmd(api, temp = 0, isQueued = 1)

    # !! Partie à compléter pour permettre le passage du repère Feuille au repère de base du robot
    # calculer le vecteur O_0 O_F (de dimension 3x1) dans le repère de base du robot (R_0)
    # O_0_O_F = numpy.array([[??], [??], [??]])  # en mm

    x_f = 193.85
    y_f = -129.14
    z_f = -40.73
    O_0_O_F = numpy.array([[x_f], [y_f], [z_f]])

    # calculer la matrice de rotation R (de dimension 3x3) permettant le passage de R_0 à R_Feuille
    # x de r0 est y de rfeuille
    #y de r0 est x de rfeuille
    
    R = numpy.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])

    #Async PTP Motion
    # i permet de considérer l'ensemble des cercles détectés
    for i in center_mm:
        center_cm_R_0 = numpy.add(O_0_O_F, numpy.dot(R, numpy.array([[i[0]], [i[1]], [0]])))
        lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, center_cm_R_0[0], center_cm_R_0[1], center_cm_R_0[2], 0, isQueued=1)[0]
        lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 250, 0, 50, 0, isQueued=1)[0]

    #Start to Execute Command Queued
    dType.SetQueuedCmdStartExec(api)

    #Wait for Executing Last Command 
    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(100)

    #Stop to Execute Command Queued
    dType.SetQueuedCmdStopExec(api)

#Disconnect Dobot
dType.DisconnectDobot(api)
