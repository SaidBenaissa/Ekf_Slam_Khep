/*
* fichier                   : TRAJET.C
* fonction                  : generation d'un fichier de consignes de reference avec des 
*                             spirales cubiques pour les robots du type de MELODY. 
* date de creation          : mai 1997
* date de derni_re r_vision :
* auteur                    : J. ROBIN
*/

/************************ Fichiers d'inclusions *********************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <math.h>
#include <ctype.h>


/************************ Les constantes ****************************/
#define false              0
#define true               !false
#define PI                 3.14159265359
#define rayon				0.095
#define demi_voie			0.105

#define NB_MAX_POSTURES_IN      20/* nb max de postures du fichier d'entree ou entree au clavier */
#define NB_MAX_POSTURES         40/* nb max de postures entrees+intermediaires. Au plus 2*entrees */
#define POSTURE_SYMETRIQUE      1 /* les postures n-1 et n sont symetriques */ 
#define POSTURE_PARALLELE       2 /* les postures n-1 et n sont non symetriques et paralleles */ 
#define POSTURE_NON_SYMETRIQUE  3 /* les postures n-1 et n sont non symetriques et non paralleles*/ 

#define PERIOD         (float)  5.0e-3 /* periode d'echantillonnage pour fichier consigne */
                                     /* doit etre la meme que celle utilisee par la     */
                                     /* commande du robot                               */
#define REDUCT                  0.001
#define GOLD                    (sqrt(5.5)-1.0)/2.0

/******************** Les definitions des types *********************/
typedef struct {
    double        x_consigne, y_consigne, teta_consigne, vitesse_consigne, omega_consigne;
} enregistrement;


/********************** Les variables globales **********************/

/***** variables pour recuperation des donnees sur les postures *****/
FILE            *fichier_postures;

char            nom_fichier[80];
double          x_in[NB_MAX_POSTURES_IN] = {0.0};
double          y_in[NB_MAX_POSTURES_IN] = {0.0};
double          teta_in[NB_MAX_POSTURES_IN] = {0.0};
double          vit_in[NB_MAX_POSTURES_IN] = {0.0};
double          omega_in[NB_MAX_POSTURES_IN] = {0.0};
unsigned        nb_tot_postures_in; /* nombre de postures entrees */ 

/***** variables pour le calcul des postures intermediaires *********/
int             typedeposture;
double          x[NB_MAX_POSTURES] = {0.0};
double          y[NB_MAX_POSTURES] = {0.0};
double          teta[NB_MAX_POSTURES] = {0.0};
double          vit[NB_MAX_POSTURES] = {0.0};
double          omega[NB_MAX_POSTURES] = {0.0};

/** variables pour le calcul des parametres des segments de chemin **/
double          d;          /* distance cartesienne entre les postures i et i+1 */
double          L;          /* longueur du segment entre 2 postures i et i+1*/
double          Da;         /* taille standard */
double          alpha;      /* variation de cap entre les postures i et i+1 */
double          coef_courb; /* coefficient de courbure du segment entre 2 postures i et i+1*/
double          longueur;   /* distance entre le depart et la fin du segment i */
double          longueur_p; /* distance entre le depart et la fin du segment i-1 ou debut i */
double          temps_f;    /* duree du segment i */
double          accel_max;  /* acceleration max sur le segment i */

unsigned        nb_tot_postures; /* nb de postures de la trajectoire complete entree+intermediaires */ 
unsigned        nb_chemins; /* nombre de chemins de la trajectoire complete = nb_tot_postures-1 */ 
unsigned        chemin; /* numero du chemin i */

double          vitesse_consigne1, vitesse_consigne2, vart1;

/* variables de sortie de la partie calcul des parametres des segments de chemin */


double          uf[NB_MAX_POSTURES] = {0.0};         /* duree du segment i */ 
double          Kc_i[NB_MAX_POSTURES] = {0.0};       /* coefficient de courbure du segment i */
double          l3_sur_12[NB_MAX_POSTURES] = {0.0};
double          Acmax[NB_MAX_POSTURES] = {0.0};      /* acceleration max sur le segment i */
double          longueur_i[NB_MAX_POSTURES] = {0.0}; /* distance depart a fin du segment i */
double          alpha_i[NB_MAX_POSTURES] = {0.0};    /* alpha du segment i */
double          temps_total;                         /* duree de la trajectoire complete */
unsigned long   n;                                   /* n = nombre de periode d'echantillonnage */ 
                                                     /* pour avoir le temps desire */

double          longueur_initiale[NB_MAX_POSTURES] = {0.0};
double          longueur_initiale[];
double          vitesse_initiale[NB_MAX_POSTURES] = {0.0}; /* vitese au debut de chaque segment */ 
double          vitesse_initiale[];                        /* vitesse au debut de chaque segment */ 
double          teta_initiale[NB_MAX_POSTURES] = {0.0};
double          teta_initiale[];

double          uf[];               /* duree du segment i */ 
double          Kc_i[];             /* coefficient de courbure du segment i */
double          l3_sur_12[];
double          Acmax[];            /* acceleration max sur le segment i */
double          longueur_i[];       /* distance depart a fin du segment i */
double          alpha_i[];          /* alpha du segment i */

/************* variables de l'interpolateur *************************/
double          temps, temps_i;
double          acc_cons;
double          u, temps_fin;
double          x_consigne, y_consigne, teta_consigne, vitesse_consigne, omega_consigne;
double          absc_i, abscisse_cur;
double          courbure, d_courb;

/******* Variables pour tableau de consigne *************************/
//enregistrement  huge *mouchard_consigne;
unsigned long   tableau_debut,tableau_fin,tableau_plein; /*index du tableau*/
unsigned        taille,premier_enregistrement;

char            sauvegarde[80] = "G:\\CONSIGNE.DAT";
                /* ou se fait la sauvegarde du fichier des consignes, fichier de sorties */
                
/********************* Divers****************************************/
unsigned        i,j,OK,choix;



/*************** initialisation des variables ***********************/
void initialise(void)
{
  nb_tot_postures_in =0;
  nb_tot_postures=0;
  nb_chemins=0;
  temps=temps_i=0.0; 
  acc_cons=0.0;
  typedeposture=0;

  abscisse_cur=0.0;
  absc_i=0.0;
  courbure=0.0;
  d_courb=0.0;
 
  x_consigne=0.0;
  y_consigne=0.0;
  teta_consigne=0.0;
  vitesse_consigne=0.0;
  vitesse_consigne1=0.0;
  vitesse_consigne2=0.0;    
  omega_consigne=0.0;
}



/************** calcul de la taille standard ***********************/
double taille_standard(double angle)
{
unsigned int n=4,j,k,m,Nbre;
double T[5][5],x,y,h;

  h=0.5;
  for (k=0;k<=n;k++)  {
      for (j=0;j<=n;j++)  {
         T[k][j]=0.0;
      }   
  }    
  
  k=0;
  for(m=0;m<=n;m++)  {
      x=0;
      Nbre=(unsigned int)pow(2.0,(double)m);
      for(j=0;j<=Nbre;j++)   {
         y=2.0*cos(angle*x*(1.5-2.0*x*x));
         if(j==0) y=y/2.0;
         if(j==Nbre) y= y/2.0;
         T[k][m]=T[k][m]+y*h;
         x=x+h;
      }
      h=h/2.0;
  }
  
  j=1;
  for(k=1;k<=n;k++)   {
      for(m=j;m<=n;m++)  {
         T[k][m]=(pow(4.0,(double)k)*T[k-1][m]-T[k-1][m-1])/(pow(4.0,(double)k)-1);
      }   
      j++;
  }
return(T[n][n]);
}



/**********  Repositionne angle par rapport a (angle_ref +/- PI) ****/
double mod(double angle,double angle_ref)
{
  while ((angle-angle_ref)<-PI) angle+=2.0*PI;
  while ((angle-angle_ref)> PI) angle-=2.0*PI;
  return (angle);
}



/**********************procedure de calcul du cout de la spirale*****/
double calcul_cout (double angle,double c1,double c2,double R,double phi,double phi1)
{
double d1,d13,d2,d23,terme1,terme2,terme3,sin1,sin13,sin2,sin23;

  if (angle<0) {
      d1     = taille_standard (c1-phi1);
      d2     = taille_standard (c2+phi1);
      d13    = d1*d1*d1;
      d23    = d2*d2*d2;
      sin1   = sin(phi1/2.0);
      sin2   = sin((phi-phi1)/2.0);
      sin13  = sin1*sin1*sin1;
      sin23  = sin2*sin2*sin2;
      terme1 = 1.5/(R*R*R);
      terme2 = pow((c1-phi1),2.0)*d13/sin13;
      terme3 = pow((c2+phi1),2.0)*d23/sin23;
  }
  else {
      d1     = taille_standard (c1+phi1);
      d2     = taille_standard (c2-phi1);
      d13    = d1*d1*d1;
      d23    = d2*d2*d2;
      sin1   = sin(phi1/2.0);
      sin2   = sin((phi-phi1)/2.0);
      sin13  = sin1*sin1*sin1;
      sin23  = sin2*sin2*sin2;
      terme1 = 1.5/(R*R*R);
      terme2 = pow((c1+phi1),2.0)*d13/sin13;
      terme3 = pow((c2-phi1),2.0)*d23/sin23;
  }
  return (terme1*(terme2+terme3));
}



/* procedure de minimisation du cout dans le cas de postures non paralleles */
double NBDOR (double xg,double xd,double c1,double c2,double r,double angle)
{
double L,x1,x2,y1,y2,phi;
int Nmax,i;

  phi  = xd;
  L    = (xd-xg)*GOLD;
  x1   = xd-L;
  x2   = xg+L;
  y1   = calcul_cout (angle,c1,c2,r,phi,x1);
  y2   = calcul_cout (angle,c1,c2,r,phi,x2);
  Nmax = (int) (1.0+log(REDUCT)/log(GOLD));

  for (i=1;i<=Nmax;i++) {
      L = L*GOLD;
      if (y1<y2) {
              xd = x2;
              x2 = x1;
              y2 = y1;
              x1 = xd-L;
              y1 = calcul_cout (angle,c1,c2,r,phi,x1);
      }
      else {
              xg = x1;
              x1 = x2;
              y1 = y2;
              x2 = xg+L;
              y2 = calcul_cout (angle,c1,c2,r,phi,x2);
      }
  } /* fin du for */
  return ((x1+x2)/2.0);
}



/*** Renvoi le type des postures: Symetriques , Paralleles , Non symetriques et non paralleles ***/
int type_de_posture (double x_deb, double y_deb, double theta_deb,
                     double x_fin, double y_fin, double theta_fin)
{
double beta,test;

  if (x_fin==x_deb) {
      beta= PI/2.0;
  }
  else {
      beta = atan2(y_fin-y_deb,x_fin-x_deb);
  }
 
  test = fabs( mod( (2.0*beta)-theta_deb-theta_fin , 0.0) );
  
  if (test <= (PI/180))      {  /* moins de 1 degre soit 0.017 rd */
      printf("\r\nP.O");
      return (POSTURE_SYMETRIQUE);
  }
  else   {
      if ( mod (theta_deb,0.0) == mod(theta_fin,0.0) )  {
          printf("\r\nP.I");      
          return(POSTURE_PARALLELE);
      }
      else{
          printf("\r\nP.I");      
          return(POSTURE_NON_SYMETRIQUE);
      } /* fin du deuxieme if */
  } /* fin du premier if */
}



void calcul_point_int (double x_deb,double y_deb,double theta_deb,
                        double x_fin,double y_fin,double theta_fin,
                        double *x_int,double *y_int,double *theta_int,
                        int typedeposture)
{
double alpha,beta,d,c,xc,yc,r1,r2,r3,r4,taux,phi,R;
double d1,c1,c2,phi1,gamma,beta1;

  switch (typedeposture) {

      case POSTURE_PARALLELE:
         beta = atan2(y_fin-y_deb,x_fin-x_deb);

         /* Pt intermediaire = Pt milieu */
         *x_int = (x_deb+x_fin)/2.0;
         *y_int = (y_deb+y_fin)/2.0;
         *theta_int = 2.0*beta-theta_deb;
      break;

      case POSTURE_NON_SYMETRIQUE:
         d=sqrt(pow((x_fin-x_deb),2.0)+pow((y_fin-y_deb),2.0));
         alpha=(theta_fin-theta_deb);

         c=(cos(alpha/2.0))/sin(alpha/2.0);
         xc=(x_fin+x_deb+c*(y_deb-y_fin))/2.0;
         yc=(y_fin+y_deb+c*(x_fin-x_deb))/2.0;
         r1=pow(xc,2.0);
         r2=pow(yc,2.0);
         r3=(x_fin*x_deb+y_fin*y_deb);
         r4=((y_fin*x_deb)-(y_deb*x_fin))*c;
         R=sqrt(r1+r2-r3+r4);
         
         if ( R/d <= 0.5 ) {
            taux=0.0;
         }
         else {
            taux=sqrt(4.0*pow(R/d,2.0)-1.0);
         }


         if (taux==0.0)
            phi=PI;                 
         else
            phi=2.0*atan(1.0/taux);

         gamma = atan2 (yc-y_deb,xc-x_deb);


         if (alpha<0.0) {

             c1=mod(PI+2.0*(gamma-theta_deb),0.0);
             c2=alpha-c1;
             phi1=NBDOR(0.0,phi,c1,c2,R,alpha);
             *theta_int=c1-phi1+theta_deb;

         } else {

             c1=mod(-PI+2.0*(gamma-theta_deb),0.0);
             c2=alpha-c1;
             phi1=NBDOR(0.0,phi,c1,c2,R,alpha);
             *theta_int=c1+phi1+theta_deb;
         }

         beta1=(*theta_int+theta_deb)/2.0;
         d1=2.0*R*sin(phi1/2.0);
         *x_int=x_deb+d1*cos(beta1);
         *y_int=y_deb+d1*sin(beta1);

      break;
  }

}



/************** CORPS DU PROGRAMME PRINCIPAL  ***********************/

void main(void)
{

  initialise(); 
  


  /*** transfert des parametres et lancement de l'application *******/
  do {
  
	  /* Test and calculated (if necessary) the intermediate positions. Filled the table postures
	  * Used later to calculate the trajectories. Beforehand, the angles we pass
	  * Teta in radians.
	  */

	  /* Initialize the loop for filling the table of all postures */
  for (i=0;i<nb_tot_postures_in;i++)    {
          teta_in[i] = teta_in[i]*PI/180.0; 
  }
  /* a partir de maintenant les angles teta sont en radian */

  /* affichage des valeurs de la posture 0 */
  i=j=0;
  x[j] = x_in[i]; y[j] = y_in[i]; teta[j] = teta_in[i];
  vit[j] = vit_in[i]; omega[j] = omega_in[i];
  printf("\r\nLes postures a joindre par spirales cubiques sont :");  
  printf("\r\n    P.O=Posture d'Origine    -    P.I=Posture Intermediaire\n");
  printf("\r\nP.O %u x=%8.3lf y=%8.3lf teta=%8.3lf v=%8.3lf w=%8.3lf",
                 j,x[j],y[j],teta[j],vit[j],omega[j]);
  /* fin d'affichage des valeurs de la posture 0 */                
  

  /* on execute la boucle pour toutes les postures du tableau _in */ 
  for(i=1;i<nb_tot_postures_in;i++)
  {
  
        typedeposture=type_de_posture(x_in[i-1],y_in[i-1],teta_in[i-1],x_in[i],y_in[i], teta_in[i]);
                                          
        if (typedeposture == POSTURE_SYMETRIQUE)
        {
              j++;
              x[j]     = x_in[i];
              y[j]     = y_in[i];
              teta[j]  = teta_in[i];
              vit[j]   = vit_in[i];
              omega[j] = omega_in[i];

        }
        
        else
        {
              j++;
              calcul_point_int (x_in[i-1] , y_in[i-1] , teta_in[i-1] ,
                                x_in[i]   , y_in[i]   , teta_in[i]   ,
                                &x[j]     , &y[j]     , &teta[j]      ,
                                typedeposture);
              vit[j] = ( vit_in[i-1] + vit_in[i] ) / 2.0;                
              omega[j] = ( omega_in[i-1] + omega_in[i] ) / 2.0;

              j++;                  
              x[j] = x_in[i]; y[j] = y_in[i]; teta[j] = teta_in[i];
              vit[j] = vit_in[i]; omega[j] = omega_in[i];                                

        } /* fin du if */                       

        nb_tot_postures = j+1;
        
  } /* fin du for */      
  


  /***** Calculated parameters in spiral **********************************/
  longueur_p = longueur = 0.0;
  temps_total=0.0;
  printf("\r\n\nParametres des chemins :\n");
  
  for(i=1;i<nb_tot_postures;i++)
  {
        /* distance euclidienne entre les postures i-1 et i */
        d = sqrt(pow((x[i]-x[i-1]),2.0)+pow((y[i]-y[i-1]),2.0)); 
        
        /* deviation de la posture i par rapport a la posture i-1 */
        alpha = (teta[i]-teta[i-1]);          /* on est en radian */
        if ( fabs(alpha) > PI )
        {
             if ( teta[i] > teta[i-1] )
             {
                  alpha = PI - fabs(alpha); 
             }
             if ( teta[i] < teta[i-1] )
             {
                  alpha = fabs(alpha) - PI; 
             }
        }

        /* Da = D(alpha) = distance standard de la spirale cubique */
        Da = fabs(taille_standard(alpha));
        
        /* longueur du chemin entre les postures i-1 et i */
        L = d/Da;
        
        /* le coefficient de courbure sert pour le calcul de teta */
        coef_courb = 6*alpha/(L*L*L);
        
        /* longueur = abcisse curviligne de la posture i */
        longueur   = longueur+L;

         /* calcul des parametres de l'interpolateur */
        temps_f    = 2.0*(longueur-longueur_p)/(vit[i]+vit[i-1]);
        accel_max  = (3.0/4.0)*(pow(vit[i],2.0)-pow(vit[i-1],2.0)) / (longueur-longueur_p);
        temps_total+=temps_f;
        

        
        /********** sauvegarde pour generation de mouvement *********/
        vitesse_initiale[i]  = vit[i-1];
        longueur_i[i]        = L;
        longueur_initiale[i] = longueur_p;        
        teta_initiale[i]     = teta[i-1];    
        uf[i]                = temps_f;
        Kc_i[i]              = coef_courb;
        Acmax[i]             = accel_max;
        alpha_i[i]           = alpha;
        

        /* longueur_p est l'abcisse curviligne de la posture i-1. On ajoute la longueur L,
        *  longueur du dernier chemin a la longueur precedente pour le calcul de la prochaine
        *  boucle.
        */
        longueur_p = longueur;
  }/*fin du for */


 /** fin du calcul des parametres des spirales et mise en memoire **/
  
  
  x_consigne    = x[0];
  y_consigne    = y[0];
  teta_consigne = teta[0];
  premier_enregistrement=1;/* pour ecraser l'ancien consigne.dat a la premiere ecriture */
  
  for (chemin=1;chemin<=nb_tot_postures;chemin++) { 
      temps_fin = uf[chemin]+(PERIOD/2);
      printf("\r\nCalcul des parametres du chemin numero %d en cours.",chemin);
      
      for ( temps_i=PERIOD ; temps_i < temps_fin ; temps_i = temps_i+PERIOD )  {
         
         /* Polynome de degre 4 */
         u=temps_i/uf[chemin];
         
         acc_cons = 4.0*Acmax[chemin]*u*(1.0-u);    
         
         vitesse_consigne = 4.0*Acmax[chemin]*uf[chemin]*u*u*(0.5-u/3.0)
                             +vitesse_initiale[chemin];
         
         abscisse_cur=4.0*Acmax[chemin]*uf[chemin]*uf[chemin]*u*u*u*(1.0/6.0-u/12.0)
                             +vitesse_initiale[chemin]*temps_i+longueur_initiale[chemin]; 
         
         /* longueur relative a la portion i de la spirale */
         absc_i = abscisse_cur-longueur_initiale[chemin];
         
         /* courbure relative a la portion i de la spirale */
         courbure=Kc_i[chemin]*absc_i*(longueur_i[chemin]-absc_i);
         
         /* derivee de la courbure par rapport a la longueur relative absc_i */
         d_courb =Kc_i[chemin]*(longueur_i[chemin]-2.0*absc_i);
         
         /* vitesse angulaire du robot */ 
         omega_consigne = vitesse_consigne * courbure;
         
         /* Orientation du robot */          
         teta_consigne = teta_initiale[chemin] +
               Kc_i[chemin] * ( (longueur_i[chemin]/2.0-absc_i/3.0) * absc_i * absc_i );
         
         /* Coordonnees du repere liee du robot */               
         x_consigne = x_consigne + vitesse_consigne*cos(teta_consigne)*PERIOD;
         y_consigne = y_consigne + vitesse_consigne*sin(teta_consigne)*PERIOD;
         
         /* vitesses articulaires*/
         vart1 = demi_voie * courbure;
         

      } /* fin du premier for */
  }

 }while(true);
 
}
