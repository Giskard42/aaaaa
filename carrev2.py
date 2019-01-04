from tkinter import *
from math import *
from time import *
from numpy import *
from pdb import *
import matplotlib.pyplot as plt

def sommet(self):
    x=float(self.pos[0])
    y=float(self.pos[1])
    d=float(self.d)
    theta=float(self.theta)
    return([x+d*(cos(theta)+sin(theta)), y-d*(cos(theta)-sin(theta)), x-d*(cos(theta)-sin(theta)), y-d*(cos(theta)+sin(theta)), x-d*(cos(theta)+sin(theta)), y+d*(cos(theta)-sin(theta)), x+d*(cos(theta)-sin(theta)), y+d*(cos(theta)+sin(theta))])

def sommet_aff(self):
    x=float(self.pos[0])
    y=canvas_height - float(self.pos[1])
    d=float(self.d)
    theta=float(self.theta)
    return([x+d*(cos(theta)+sin(theta)), y-d*(cos(theta)-sin(theta)), x-d*(cos(theta)-sin(theta)), y-d*(cos(theta)+sin(theta)), x-d*(cos(theta)+sin(theta)), y+d*(cos(theta)-sin(theta)), x+d*(cos(theta)-sin(theta)), y+d*(cos(theta)+sin(theta))])

class Carre:
    def __init__(self, m, d, pos, v, theta, w):
        self.m = float(m) #masse
        self.I = float(m*d*d)/6
        self.pos = array(pos) #coordonnées pos= array([x,y])
        self.v = array(v) #vitesses initiales v=array([vx, vy])
        self.theta = theta #angle initial par rapport à l'est (sens trigo)
        self.w = w #vitesse angulaire initiale
        self.d = d #largeur du carre
        self.sommets = sommet(self) #points du carré
        self.L = [[0,pos,theta]] #historique des positions et orientations
        #affichage graphique


    def affiche(self):
        return()
    
    def deplace(self, t, pos, v, theta, w): #permet de changer les caractéristiques du carre arbitrairement
        self.pos = pos
        self.v = v
        self.theta = theta
        self.w = w
        self.sommets= sommet(self)
        self.L.append([t,pos,theta])
        #affichage graphique 
    
    def avance(self, dt, v, w): #fait avancer le carre. a faire appres le TRC_TMC
        self.v = v
        self.w = w
        self.pos = self.pos + dt * self.v
        self.theta = self.theta + dt * w
        self.sommets= sommet(self)

class Droite:
    def __init__(self, y, theta):
        self.y = y #Ordonnée à l'origine
        self.theta = theta #Angle avec l'Est (trigo)
        if theta !=pi/2 and theta != -pi/2:
            self.a = tan(theta)
        else:
            self.a=0 #coefficient directeur de la droite
        #affichage graphique


    def affiche(self):
        return()
    
    def deplace(self, y, theta): #permet de déplacer la droite
        self.y = y
        self.theta = theta
        #affichage graphique 
    

def poids_carre(theta,dt,vx,vy):
    a=9,81
    vx-=a*sin(theta)*dt
    vy-=a*cos(theta)*dt
    return([vx,vy])
    #le moment du poids est nul donc w est constant

def detection_collision_droite_carre(d,c): #,dist
    dist=abs(c.pos[1]-d.a*c.pos[0]-d.y)/(sqrt(1+d.a**2)) #distance entre le centre du carre et la droite #a mettre dans la fonction principale
    if dist>c.d*sqrt(2):
        return(False)
    else:
        return(dist <= c.d*(abs(cos(c.theta-d.theta))+abs(sin(c.theta-d.theta))))  # dist <= demi-hauteur du carre vis a vis de la droite

def recalage_droite_carre(d,c,eps): #,d
    if abs(cos(c.theta-d.theta)+sin(c.theta-d.theta)) >= abs(cos(c.theta-d.theta)-sin(c.theta-d.theta)):
        theta=acos(abs(cos(c.theta-d.theta)+sin(c.theta-d.theta))/sqrt(2))
    else:
        theta=acos(abs(cos(c.theta-d.theta)-sin(c.theta-d.theta))/sqrt(2))
    h=abs(c.pos[1]-d.a*c.pos[0]-d.y)/(sqrt(1+d.a**2)) #a enlever plus tard, dist centre carré-droite
    n = array([-sin(d.theta), cos(d.theta)])
    VN_P=(inner(c.v,n))/(c.d*sqrt(2)) #vitesse . normale
    H=h/(c.d*sqrt(2))
    if c.w==0: #Cas où le déplacement est rectiligne
        dt=(H-cos(theta))/VN_P
    else:
        dt=dicho(lambda t: cos(theta-c.w*t) + VN_P*t-H, (theta - pi/2)/c.w, theta/c.w, eps)
    c.pos = c.pos - dt * c.v
    c.theta -= dt * c.w #Retour au point de contact
    return(dt)

def dicho(f,a,b,eps):
    if a>b:
      a,b=b,a  
    while abs(a-b)-eps>0:
        m=(b+a)/2
        if f(a)*f(m)>=0:
            a=m
        else:
            b=m
    return b

def reaction_droite_carre(d,c):
    return()

def trouver_sommet_impact_droite_carre(c,d):#a utiliser apres le recalage
    s=[c.sommets[0],c.sommets[1]]
    max=abs(c.sommets[1]-d.a*c.sommets[0]-d.y)
    for i in range(1,4):
        if abs(c.sommets[2*i+1]-d.a*c.sommets[2*i]-d.y) > max :
            s=[c.sommets[2*i],c.sommets[2*i+1]]
    return(array([s[0]/(sqrt(1+d.a*d.a)),s[1]/(sqrt(1+d.a*d.a))]))

def rep_collision_droite_carre(c,d):#la normale est de carre vers droite
    e = 0.8 #coeff restitution
    s = trouver_sommet_impact_droite_carre(c,d)#coord du point de collision
    n = array([sin(d.theta),-cos(d.theta)])
    r = s-c.pos #vecteur du centre vers le point de contact 
    vr = (c.v + rot_90(c.w * r))
    J = (1+e) * inner(vr,n) / ( 1/c.m + ((r[0]*n[1])**2 + (r[1]*n[0])**2)/(2*c.I)) #impulsion
    c.v = c.v - (J/c.m)*n
    c.w-= J/c.I*p_v(r,n) #change les valeurs de vitesse et rotation
    return()
    
def p_v(a,b):
    return(a[0]*b[1]-a[1]*b[0])
def rot_90(a):
    return(array([-a[1],a[0]]))

def friction_droite_carre(c,d):
    mu_s=0.3
    mu_d=0.4
    if inner(vr,n) != 0 :
        t=(vr-p_v(vr,n)*n)/abs(vr-p_v(vr,n)*n)
        if p_v(vr,t) == 0 or m*vr*t <= mu_s*J:
            Jt=-(m*p_v(vr*t))*t
        else:
            Jt=-mu_d*t
    else:
         Jt=array([0,0])
    return(Jt)

z=0
def rep():
    global z
    z=1

def debug():
    global z,pos,v,w,deltat
    main.delete(pos,v,w,dt)
    pos = main.create_text(100,25, text='pos='+str(c.pos), fill='black')
    v = main.create_text(100,40, text='v='+str(c.v), fill='black')
    w = main.create_text(100,55, text='w='+str(c.w), fill='black')
    deltat = main.create_text(75,70, text='dt='+str(dt), fill='black')
    main.update()
    if z !=0:
        z=0
    else:
        z=1
        debug()
    
def courbe(c):
    T,X,Y,Theta=[],[],[],[]
    for i in c.L:
        T.append(i[0])
        X.append(i[1][0])
        Y.append(i[1][1])
        Theta.append(i[2])
    T=array(T)
    X=array(X)
    Y=array(Y)
    Theta=array(Theta)
    plt.plot(T, Y, "r", label="Y")
    plt.plot(T, Theta, "b", label="Theta")
    plt.show()

#test des fonctions detection_collision_droite_carre et recalage_droite_carre ainsi que l'affichage graphique
#Pas encore de réaction physique fonctionnelle
for i in range(1):
    canvas_width = 2000
    canvas_height =1280
    python_green = "#476042"
    master = Tk()
    
    main = Canvas(master, width=canvas_width, height=canvas_height, bg= 'blue')
    B = Button(master, text='continuer', command=rep)
    fen = main.create_window(300,100, window=B)
    main.pack()
    
    
    # 3 param à modifier
    c=Carre(10,30,[1100.0,1200.0],[10.0,0.0], 1.0 , 0.1)
    d=Droite(500,pi/9)
    dt=0.1
    
    pos,v,w,deltat=0,0,0,0
    Em= c.m*inner(c.v,c.v)/2+c.I*c.w**2/2+c.m*c.pos[1]*9.80665
    text = main.create_text(100,10, text='Em='+str(Em), fill='black')
    main.create_line(0,canvas_height-d.y,canvas_width,canvas_height-canvas_width*d.a-d.y, width=1)
    points=sommet_aff(c)
    a=main.create_polygon(points, outline='red', fill='yellow', width=1)
    debug()
    main.pack()
    master.mainloop()
    eps=10**(-7)
    for i in range(10**8):
        if c.w != 0:
            dt=min(dt,abs(pi/(3*c.w)))
        main.after(0)
        c.v[1] -= 9.80665*dt
        #c.w *= 0.99a
        c.avance(dt,c.v,c.w)
        '''print(sign(Em-c.m*inner(c.v,c.v)/2+c.I*c.w**2/2+c.m*c.pos[1]*9.80665))'''
        Em = c.m*inner(c.v,c.v)/2+c.I*c.w**2/2+c.m*c.pos[1]*9.80665
        main.delete(text)
        text = main.create_text(100,10, text='Em='+str(Em), fill='black')
        if detection_collision_droite_carre(d,c):
            #print(c.pos,c.v)
            dt2=recalage_droite_carre(d,c,eps) #Remise au point de contact
            a=main.create_polygon(sommet_aff(c), outline='red', fill='green', width=1)
            c.L.append([c.L[-1][0]+dt-dt2,c.pos,c.theta])
            rep_collision_droite_carre(c,d) #chgt des vitesses au point de contact
            c.avance(dt,c.v,c.w)
        main.create_polygon(sommet_aff(c), outline='red', fill='yellow', width=1)
        c.L.append([c.L[-1][0]+dt,c.pos,c.theta])
        #set_trace()
        debug()
        main.update()


    
#fin test
'''
canvas_width = 200
canvas_height =200
python_green = "#476042"
master = Tk()

main = Canvas(master, width=canvas_width, height=canvas_height)
main.pack()


points = [3,3,canvas_width/2,3, canvas_width/2, canvas_height/2,3,canvas_height/2]
a=main.create_polygon(points, outline=python_green, 
            fill='yellow', width=3)

mainloop()
points2 = [50,50,canvas_width,canvas_height/2, 0, canvas_height]
main.create_polygon(points2, outline=python_green, 
            fill='yellow', width=3)
main.coords(a,5,5,30,70,50,50)

def test(a):
    return(a.x,a.y,a.vx,a.vy,a.theta,a.w,a.m,a.d)
'''

""" Trucs réutilisables
r1=(cos(c.theta),sin(c.theta))
r2=(-sin(c.theta),cos(c.theta)) #base locale du carre
"""