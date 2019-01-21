from tkinter import *
from math import *
from time import *
from numpy import *
from pdb import *
#import matplotlib.pyplot as plt

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

def dist(c,d):
    BH = (c.pos[0]*cos(d.theta)+(c.pos[1]-d.y)*sin(d.theta)) #distance entre l'origine de la droite et le projeté orthogonal du carré sur la droite
    H = array([BH*cos(d.theta),d.y+BH*sin(d.theta)]) #projeté orthogonal du carré sur la droite
    dist = inner((c.pos - H),array([-sin(d.theta),cos(d.theta)])) # produit scalaire entre le vecteur normal reliant la droite au centre du carré et la normale au carré
    return(dist)

def detection_collision_droite_carre(d,c):
    dis = dist(c,d) # produit scalaire entre le vecteur normal reliant la droite au centre du carré et la normale au carré : Si le carré est en dessous, alors il y a collision
    if dis>c.d*sqrt(2):
        return(False)
    else:
        return(dis <= c.d*(abs(cos(c.theta-d.theta))+abs(sin(c.theta-d.theta))))  # dist <= demi-hauteur du carre vis a vis de la droite



'''def recalage_droite_carre(d,c,eps): #,d
    if abs(cos(c.theta-d.theta)+sin(c.theta-d.theta)) >= abs(cos(c.theta-d.theta)-sin(c.theta-d.theta)):
        theta=acos(abs(cos(c.theta-d.theta)+sin(c.theta-d.theta))/sqrt(2))
    else:
        theta=acos(abs(cos(c.theta-d.theta)-sin(c.theta-d.theta))/sqrt(2))
    h=abs(c.pos[1]-d.a*c.pos[0]-d.y)/(sqrt(1+d.a**2)) #a enlever plus tard, dist centre carré-droite
    n = array([-sin(d.theta), cos(d.theta)])
    VN_P=(inner(c.v,n))/(c.d*sqrt(2)) #vitesse . normale
    H=h/(c.d*sqrt(2))
    if c.w==0: #Cas où le déplacement est rectiligne
        dt2=(H-cos(theta))/VN_P
    else:
        dt2=dicho(lambda t: cos(theta-c.w*t) + VN_P*t-H, (theta - pi/2)/c.w, theta/c.w, eps)
    c.pos = c.pos - dt2 * c.v
    c.theta -= dt2 * c.w #Retour au point de contact
    return(dt)'''

def parcours(d,c,dt,eps):
    c.pos = c.L[-1][1]
    c.theta = c.L[-1][2]
    t=dt
    while dist(c,d) >= c.d*(abs(cos(c.theta-d.theta))+abs(sin(c.theta-d.theta)))+eps or dist(c,d) <= c.d*(abs(cos(c.theta-d.theta))+abs(sin(c.theta-d.theta)))-eps or dt>10**(-4):
        c.avance(dt,c.v,c.w)
        if detection_collision_droite_carre(d,c):
            dt=-abs(dt/2)
        else:
            dt=abs(dt/2)
        t-=dt
        """a=main.create_polygon(sommet_aff(c), outline='red', fill='purple', width=1)
        main.update()
        debug()"""
    return(t)

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

def trouver_sommet_impact_droite_carre(c,d):#a utiliser apres le recalage: donne les coordonnées du point de contact
    s=[c.sommets[0],c.sommets[1]]
    min=c.sommets[1]-d.a*c.sommets[0]-d.y
    for i in range(1,4):
        if c.sommets[2*i+1]-d.a*c.sommets[2*i]-d.y < min :
            s=[c.sommets[2*i],c.sommets[2*i+1]]
    return(array(s))

def rep_collision_droite_carre(c,d):#La formule de J est tirée de -e=(vitesse du point de contact finale selon la normale)/(vitesse du point de contact initiale selon la normale)
    mu_s=0.03
    e = 0.8 #coeff restitution
    s = trouver_sommet_impact_droite_carre(c,d)#coord du point de collision
    n = array([-sin(d.theta),cos(d.theta)])
    r = s-c.pos #vecteur du centre vers le point de contact 
    vr = (c.v + rot_90(c.w * r)) #•vitesse du point de contact
    J = (1+e) * (inner(c.v,n) + p_v(r,n)*c.w)/ ( 1/c.m + (inner(p_v(r,n),p_v(r,n)))/c.I) #ici J est sense etre negatif
    #J = (1+e) * inner(vr,n) / ( 1/c.m + ((r[0]*n[1])**2 + (r[1]*n[0])**2-2*r[0]*r[1]*n[0]*n[1])/(c.I)) #impulsion
    print(inner(vr,n),vr,r)
    print(c.v,c.w,J) 
    c#.v = c.v - (J/c.m)*(n+mu_s*rot_90(n)) formule avec frottements (lois de Coulombs)
    c.v = c.v - (J/c.m)*n
    c.w-= J/c.I*p_v(n,r) #change les valeurs de vitesse et rotation
    print(c.v,c.w)
    #debug()
    return()

def rep_collision_droite_carre2(c,d):#La formule de J est tirée de e^2=(Energie cinétique finale )/(Energie cinétique initiale)
    e = 0.5 #coeff restitution
    s = trouver_sommet_impact_droite_carre(c,d)#coord du point de collision
    n = array([-sin(d.theta),cos(d.theta)])
    t = array([cos(d.theta),sin(d.theta)])
    r = s-c.pos #vecteur du centre vers le point de contact 
    vr = (c.v + rot_90(c.w * r))
    vi2=inner(c.v,c.v) #v**2
    rn = p_v(r,n) #prod vect de r et n
    vn = inner(c.v,n) #prod scal de v et n
    #J=(sqrt(abs(c.I*c.m*(-rn**2*c.m**2*vi2+e*(c.I+c.m*rn**2)*(c.I+c.w**2+c.m**vi2)-2*c.I*c.m*(rn*c.w*vn-0.5*(vi2-vn**2)))))-c.I*c.m*(rn*c.w+vn))/(c.I+c.m*rn**2)
    J = (sqrt(abs(-1/e*(1/e*(c.I**2*c.w**2-2*c.I*c.m*rn*c.w*vn-0.5*(vi2-vn**2)+c.m**2*rn**2*vi2)-(c.I+c.m*rn**2)*(c.I*c.w**2+c.m*vi2))*c.I*c.m))-1/e*c.I*c.m*(rn*c.w+vn))/(1/e*(c.I+c.m*rn**2)) #impulsion ne marche pas: pb de signe
    #debug()
    print(c.v,c.w,J) #ici J est sense etre positif
    c.v += (J/c.m)*n
    c.w += J/c.I*p_v(r,n) #change les valeurs de vitesse et rotation
    #c.v += friction_droite_carre(c,d,vr,J,n)/c.m*t
    #c.w += p_v(r,friction_droite_carre(c,d,vr,J,n)/c.I)
    print(c.v,c.w)
    debug()
    return()

'''def rep_collision_droite_carre3(c,d):
    e = 0.3 #coeff restitution
    s = trouver_sommet_impact_droite_carre(c,d)#coord du point de collision
    n = array([-sin(d.theta),cos(d.theta)])
    r = s-c.pos #vecteur du centre vers le point de contact 
    vr = (c.v + rot_90(c.w * r))
    vi2=inner(c.v,c.v) #v**2
    rn = p_v(r,n) #prod vect de r et n
    vn = inner(c.v,n) #prod scal de v et n
    J = -(1+e)*vn/(1/c.m+rn**2/c.I) #impulsion
    debug()
    print(c.v,c.w,J)
    c.v += (J/c.m)*n
    c.w += J/c.I*p_v(r,n) #change les valeurs de vitesse et rotation
    #•c.v += friction_droite_carre(c,d,vr,J,n)
    print(c.v,c.w)
    debug()
    return()'''
   
    
def p_v(a,b):
    return(a[0]*b[1]-a[1]*b[0])
def rot_90(a):
    return(array([-a[1],a[0]]))

def friction_droite_carre(c,d,vr,J,n): #calcule les lois de Coulomb. ce n'est pas encore integre au reste du programme
    mu_s=0.3
    mu_d=0.4
    t=(vr-inner(vr,n)*n)/sqrt(inner((vr-inner(vr,n)*n),(vr-inner(vr,n)*n)))
    if inner(vr,t) == 0 or c.m*(inner(vr,t)) <= mu_s*J :
        Jt=-(c.m*inner(vr,t))*t
        print('Y')
    else:
        Jt=-mu_d*J*t
        print('Y2')
    return(Jt)

z=0
def rep():
    global z
    z=1


#debug permet d'afficher les grdeurs en jeu pendant la simulation
def debug():#rajouter le nom du label dans les 2 lignes du dessous et initialiser le label à 0
    global z,pos,v,w,deltat,epss,zz,distt,col
    main.delete(pos,v,w,deltat,epss,zz,distt,col)
    pos = main.create_text(100,25, text='pos='+str(c.pos), fill='black')
    v = main.create_text(100,40, text='v='+str(c.v), fill='black')
    w = main.create_text(100,55, text='w='+str(c.w), fill='black')
    deltat = main.create_text(75,70, text='dt='+str(dt), fill='black')
    epss = main.create_text(75,85, text='eps='+str(eps), fill='black')
    zz = main.create_text(100,100, text='z='+str(z), fill='black')
    distt = main.create_text(100,115, text='dist='+str(dist(c,d)), fill='black')
    col = main.create_text(100,130, text='collision='+str(detection_collision_droite_carre(d,c)), fill='black')
    main.update()
    if z==0:
        #z=1 #mettre en commentaire pour le mode pas à pas
        sleep(0.1)
        debug()
    else:
        z=0
    
    
def courbe(c): #permet de tracer certaines grandeurs apres l execution du programme
    global Emm
    T,X,Y,Theta=[],[],[],[]
    for i in c.L:
        T.append(i[0])
        X.append(i[1][0])
        Y.append(i[1][1])
        Theta.append(i[2])
    T=array(T)
    X=array(X)
    Y=array(Y)
    Emm=array(Emm)
    Theta=array(Theta)
    plt.plot(T, Y, "r", label="Y")
    plt.plot(T, Theta, "b", label="Theta")
    plt.plot(T, Emm, "pink", label="Em")
    plt.show()

#test des fonctions detection_collision_droite_carre et parcours ainsi que l'affichage graphique
#Pas encore de réaction physique fonctionnelle
canvas_width = 2000
canvas_height =1280
python_green = "#476042"
master = Tk()

main = Canvas(master, width=canvas_width, height=canvas_height, bg= 'blue')
B = Button(master, text='continuer', command=rep)
fen = main.create_window(300,100, window=B)
main.pack()


# c: masse, largeur, position, vitesse, inclinaison, vitesse angulaire
# d : ordonnée à l'origine, inclinaison
# modifier la position, l'inclinaison, la vitesse angulaire du carre et l'inclinaison de la droite pour tester differentes possibilités
c=Carre(100,30.0,[1800.0,1200.0],[8.0,0.0], pi/4 , 0.0)
d=Droite(100, pi/30)
dt=0.1
eps=10**(-2)

#initialisation de l'interface graphique
pos,v,w,deltat,epss,zz,distt,col=0,0,0,0,0,0,0,0
Em= (c.m*inner(c.v,c.v)/2+c.I*c.w**2/2+c.m*c.pos[1]*9.80665)*10**(-6)
Emm=[Em]
text = main.create_text(100,10, text='Em='+str(Em), fill='black')

main.create_line(0,canvas_height-d.y,canvas_width,canvas_height-canvas_width*d.a-d.y, width=1)
points=sommet_aff(c)
a=main.create_polygon(points, outline='red', fill='yellow', width=1)
debug()
main.pack()

fin_simu=0
i=0
nb_max_dt2_egale_dt_consecutif=5

#for i in range(10**8):
while (i<10**8 and fin_simu<nb_max_dt2_egale_dt_consecutif):
    i += 1
    c.v[1] -= 9.80665*dt #action gravitationnelle
    #c.w *= 0.99a
    c.avance(dt,c.v,c.w) #le carre avance entre t et t+dt
    main.create_polygon(sommet_aff(c), outline='red', fill='pink', width=1) #affichage graphique
    main.update()#met à jour l affichage
    #debug()
    Em = (c.m*inner(c.v,c.v)/2+c.I*c.w**2/2+c.m*c.pos[1]*9.80665)*10**(-6) #les 3 lignes sont la pour calculer et afficher l 'energie mecanique du carre
    main.delete(text)
    text = main.create_text(100,10, text='Em='+str(Em), fill='black')
    if detection_collision_droite_carre(d,c): #detecte une collision
        dt2=parcours(d,c,dt,eps) #trouve le moment du contact par dichotomie et replace le carre au moment du contact
        a=main.create_polygon(sommet_aff(c), outline='red', fill='green', width=1) #affiche le carre au point de contact
        c.L.append([c.L[-1][0]+dt-dt2,c.pos,c.theta]) #met à jour l'historique des positions (pas très important)
        rep_collision_droite_carre2(c,d) #chgt des vitesses au point de contact. Il y a deux façons de répondre à la collision (selon la loi de conservation considérée): on peut choisir entre rep_collision_droite_carre(c,d) et rep_collision_droite_carre2(c,d) C'est la variable J de ces fonctions qui pose problème
        if dt2==dt:
            fin_simu += 1
    else:
        fin_simu = 0
        main.create_polygon(sommet_aff(c), outline='red', fill='yellow', width=1) #affiche le carre s'il n'y a pas de collision
    c.L.append([c.L[-1][0]+dt,c.pos,c.theta])  #met à jour l'historique des positions (pas très important)
    main.update() #met à jour l affichage
    Emm.append(Em)
   

texte_fin = main.create_text(500, 100, text='FIN de la simulation', fill='red')
main.delete(fen)
bouton_fermer = Button(master, text='Fermer', command=master.destroy)
bouton_fermer.pack()
fen2=main.create_window(500,200,window=bouton_fermer)
main.update()

master.mainloop()

