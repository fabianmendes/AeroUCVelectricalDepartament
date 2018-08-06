
import math

print('Hola, sean bienvenidos/as al programa test de baterías para Radio Control (RC),\n'
    'escrito con lenguaje Python 3.6.1 en licencia \033[1mEUPL-1.2\033[3m elaborado por, y para el\n'
    'equipo Aerodesign UCV. 2018; a través del compilador online Repl.it,\n'
    'bajo el NOMBRE: "Battery calculus work-lifetime in RC creations".\n',
    'Les da la cordial bienvenida,')
print(
    "\t\t\t\t\t\t\t\t\t\t\t", "~V. Fabian Mendes. \n\t\t\t\t\t\t\t\t\t\t\t"
    "\x1B[3mAeroUCV's Electrial Student,",
    "\n\t\t\t\t\t\t\t\t\t\t\tDepartament Electrical Chief.\x1B[3m")
print(
    '''\033[1mP.S.\x1B[3m:''',
    '\033[1mEstos resultados son de manera ideal, calculados bajo condiciones ideales.\033[3m\n'
)

try:
   while True:
     mAh = (input("Ingrese el almacenamiento de la batería [mAh]: "))
     # los miliAmphere, por hora, (mAh) de la batería!
     mAh0 = bool(mAh)
     if mAh0 != True:
       print("\nNO DEBE dejar en blanco si desea continuar.")
       continue
     elif (float(mAh) != False):
       mAh = int(mAh)
       if mAh > 99999:
         print("\nVERIFIQUE los mAh escritos, es muy grande.")
         continue
       elif mAh >= 100 or mAh < -99:
         mAh = abs(mAh)
         mAh = int(mAh)
         break
       else:
         print(
           "\nVERIFIQUE los mAh escritos, son muy pequeños \n\t\t como para una batería destinada a RC.")
         continue
     else:
       print("Por favor, ingrese únicamente en Números.")
       continue
   while True:
     nS = input("Coloque el número de Celdas (S) de su batería = ")
       # C  = int(input("Ingrese la Capacidad de la bateria [25~70C]= "))
      # Capacidad de la batería, o de explosicón según necesite el usuario (la máx, y durante pocos segundos;
        # ver especificaciones de la batería!
     nS0 = bool(nS)
     if nS0:
       if int(nS) != False and (int(nS) >= 1 or int(nS) <= 20):  #puede simplificarse solo colocando: 'if float(var):'
        # inlcuso, si mal no recuerdo, se puede delimitar los intervalos con un tal 'range' o 'for'.
         nS = int(nS)
         break
       else:
         print(
           "Por favor, ingrese únicamente en Números NATURALES [de 1~20 celdas]."
                )
         continue
     else:
       print("NO DEBE dejar en blanco si desea continuar.")
     continue
   while True:
     C = input("\tIngrese la Capacidad de DESCARGA (C) = ")
     if bool(C):
       if int(C) and (C >= 10 or C < 100):
         C = int(C)
         C = abs(C)
         break
       else:
         print(
           "Por favor, ingrese únicamente en Números NATURALES ( > 09 ) [de 25~70C]."
             )
         continue
     else:
       print("NO DEBE dejar en blanco si desea continuar.")
       continue
   while True:
     Wn = input("\n Inserte los Watts (Potencia nominal, o máx) que querrá suministrar al MOTOR [### W]= " )
     if bool(Wn) != True:  #puede invertirse, este ser 'else' y abajo 'if bool(Wn)
       print("NO DEBE dejar en blanco si desea continuar.")
       continue
     if float(Wn):  #se omite el '!= False' porque se considera y asume que esto signinica que "si es verdad.."
       Wn = int(Wn)
       if Wn > 99999:
          print("\nVERIFIQUE los Watts escritos, es muy grande.")
          continue
       else:
         if Wn >= 100 or Wn < -99:
            Wn = abs(Wn)
            Wn = int(Wn)
            hp = float(Wn) / 745.7
         elif Wn >= 1000:
           print("Registrado. Tenga en cuenta que para competencias SAE Aerodesign,",
            "\nuna de las restricciones, es que lleguen <1000w al motor. Continuemos..")
           break
         else:
            print("Por favor, ingrese únicamente en Números.")
            continue
finally:
    print("\n\t Batería de",
          str(nS) + "S (celdas) y", str(mAh),
          "mAh. Y una capacidad de descarga de",
          str(C) + "C", "\n\t\t Motor trabajando a", str(Wn),
          "Watts. A lo que es igual:",
          str(hp) + "HP aprox.")

#ahora, sabiendo que W (potencia) = V * i ; i = Ao
#Hablamos de i = W/V ; i = An. Entonces habrá que IMPRIMIR los extremos (a 23v, y 25v).
Ao = (mAh / 1000) * C  # ; La Capacidad, recordemos que hay dos:
# continua y de explosición (por unos mSeg, máx POWER).
print(
    "\t\tAmphere que podría pasar por el ESC: Ao = ",
    str(Ao) +
    "A, según los datos de la batería. \nTenga cuidado en su selección del ESC, se recomienda que el máximo sea el doble sugerido como 'Ao'."
)
# EN REALIDAD ES:
# En una hora puede dar ( )mA a una capacidad de descarga de ( )C = Ao
# Listo.

Vo = [nS * 3.6444, nS * 4.1556
      ]  # máx. Voltios en una batería; 3.7v min y 4.2 máx, o inicial.
An = []
Aptr = []
trMin = []

for x in Vo:
    # Ao = (mAh / 1000) * C
    An[x] = Wn / int(Vo[x])
    # An = float(input("\n Inserte el Amphere en el cual trabajará el MOTOR [##.0] \n Recuerde que puede estar vinculado con su ESC= "))
    # Wn = V * A # Watts (potencia) necesarios.
    Aptr[x] = Ao / An[x]  # ; A' prima igual a, "Ao" entre "A necesarios".
    # Enotnces Ap será el tiempo relativo.. Ap*60 = trMin
    trMin[x] = Aptr[x] * 60  # Ap es una CONSTANTE, NO ES Amphere; mucho menos.
    # aquí deberá haber un diferencial.

#Vo = float(inpu("Ingrese los Voltios DE LA batería [##.0])= "))
# Voltios que proporcionará la batería. Min 18v, max 22~24v
Wo = Vo * Ao  # Watts (potencia) iniciales, de carga, de la batería.
V = (input(
    "\n Ingrese los Voltios a CONSUMIR [##.0] \n ENTER si serán los mismos que INGRESÓ anteriormente= "
))
Vbool = bool(V)
if (Vbool == False):
    V = Vo
else:
    V = float(V)

td = 1000 / Wn
# "Idealmente:
# Carga de la bateria: 12V x 4A = 48W
# Consumo/hora: 12V * 0.150A = 1.8W
# Duracion de la bateria, despreciando fugas, etc: 48W/1.8W = 26.666 Horas, o 26 Horas, 40 minutos.
# Saludos! " [extraído de http://www.todopic.com.ar/foros/index.php?topic=14382.0 el 2 de Ago, 2018. 13:30]

tdMin = td * 60  # OJO con EL NOMBRE de LA VARIABLE
print("\nLa batería durará, despreciando fugas y demás, idealmente: ", tdMin,
      "Mins")
# OJO CON el nombre DE LA VARIABLE.

print(" Sabiendo que sus voltajes fueron, Vo=", Vo, "y V=", V,
      "y Amphere: Ao=", Ao, "y A (motor) entre", An[0], "y", An[1])
print("\n Hablando de unos", Wo, "W por parte de la batería, y W=", Wn,
      "necesarios del avión")
# print(Wo," y ",V) # Así se imprime. Sin ERRORes
