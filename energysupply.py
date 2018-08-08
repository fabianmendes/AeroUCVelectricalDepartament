import math

print('Hola, sean bienvenidos/as al programa test de baterías para Radio Control (R/C),\n'
      'escrito con lenguaje Python 3.6.1 en licencia \033[1mEUPL-1.2\033[3m elaborado por, y para el\n'
      'equipo Aerodesign UCV. 2018; a través del compilador Pycharm, y el online Repl.it,\n'
      'bajo el NOMBRE: "Battery calculus work-lifetime in RC creations".\n', 'Les da la cordial bienvenida,')
print("\t\t\t\t\t\t\t\t\t\t\t",
      "~V. Fabian Mendes. \n\t\t\t\t\t\t\t\t\t\t\t"
      "\x1B[3mAeroUCV's Electrial Student,",
      "\n\t\t\t\t\t\t\t\t\t\t\tDepartament Electrical Chief.\x1B[3m")
print('''\033[1mP.S.\x1B[3m:''',
      '\033[1mEstos resultados don de manera ideal, calculados bajo condiciones ideales.\033[3m\n')

try:
    while True:
        mAh = (input("Ingrese el almacenamiento de la batería [mAh]: "))
        # los miliAmphere, por hora, (mAh) de la batería!
        mAh0 = bool(mAh)
        if mAh0 != True:
            print("\nNO DEBE dejar en blanco si desea continuar.")
            continue
        else:
            if (float(mAh) != False):
                mAh = int(mAh)
                if mAh > 99999:
                    print("\nVERIFIQUE los mAh escritos, es muy grande.")
                    continue
                else:
                    if mAh >= 100 or mAh < -99:
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
            if int(nS) != False and (int(nS) >= 1 or int(nS) <= 20): #puede simplificarse solo colocando: 'if float(var):'
                        # inlcuso, si mal no recuerdo, se puede delimitar los intervalos con un tal 'range' o 'for'.
                nS = int(nS)
                break
            else:
                print("Por favor, ingrese únicamente en Números NATURALES [de 1~20 celdas].")
                continue
        else:
            print("NO DEBE dejar en blanco si desea continuar.")
            continue
    while True:
        C = input("\tIngrese la Capacidad de DESCARGA (C) = ")
        if bool(C):
            if int(C):
                C = int(C)
                if (C >= 10 or C < 100):
                    C = int(C)
                    C = abs(C)
                    break
                else:
                    print("\nPor favor, ingrese únicamente en Números NATURALES ( > 09 ) [de 25~70C].")
                    continue
            else:
                print("\Por favor, ingrese únicamente en Números NATURALES ( > 09 ) [de 25~70C].")
                continue
        else:
            print("\nNO DEBE dejar en blanco si desea continuar.")
            continue
    i = 1
    while i != 0:
        Wn = float(input("\n Inserte los Watts (Potencia nominal, o máx) que querrá suministrar al MOTOR [####W]= "))
        if Wn:
            if float(Wn):  # se omite el '!= False' porque se considera y asume que esto signinica que "si es verdad.."
                Wn = int(abs(Wn))
                if Wn > 99999:
                   print("\nVERIFIQUE los Watts escritos, es muy grande.")
                   i = 1
                else:
                   if Wn >= 100 or Wn < -99:
                        # hp = float(Wn)/745.7 # está en el finally.
                        i = 0
            elif Wn >= 1000:
                print("Registrado. Tenga en cuenta que para competencias SAE Aerodesign,",
                        "\nuna de las restricciones, es que lleguen <1000w al motor. Continuemos..")

                i = 0
            else:
                 print("Por favor, ingrese únicamente en Números.")
                 i = 1
        else:
            print("NO DEBE dejar en blanco si desea continuar.")
            i = 1

finally:
    print(
        "\n\t Batería de", str(nS)+"S (celdas) y", str(mAh), "mAh. Y una capacidad de descarga de", str(C)+"C.",
        "\n\t Motor brushless trabajando a", str(Wn), "Watts. A lo que es 'igual' o menor que:",
        str(format(float(Wn)/745.7, ".2f"))+"HP aprox.")

#ahora, sabiendo que W (potencia) = V * i ; i = Ao
#Hablamos de i = W/V ; i = An. Entonces habrá que IMPRIMIR los extremos (a 23v, y 25v).
Ao = (mAh / 1000) * C  # ; La Capacidad, recordemos que hay dos:
                   # continua, y de explosición (por unos mSeg, máx POWER).
print("\t\tAmphere que podría estar pasando por el ESC: Ao = ", str(Ao) +
      "A, según los datos de la batería. \n\tTenga cuidado en su selección del ESC,"
      " se recomienda que el máx.A sea mayor o incluso cerca del doble al valor de 'Ao'.\n")
# EN REALIDAD ES:
# En una hora puede dar ( )mA a una capacidad de descarga de ( )C = Ao
# Listo.

Vo0 = nS*4.1556  # Voltaje de la batería (Vo) iniciales (t = 0), cargado!
Vo1 = nS*3.6444  # Voltaje de la batería (Vo) a punto de descargarse ( = 1)

Vo = [int(Vo1*100), int(Vo0*100)]  # máx. Voltios en una batería; 3.7v min y 4.2 máx, o inicial.
An = []  # Ampheres que deberan poder estar pasando (requeridos..) por el ESC según (..por:) el motor!

#Aptr = []   # A prima, tiempo relativo. CONSTANTEE, definiendose más abajo, para determinar el tIEMPOrELATIVO.
trMin = []  # tiempo relativo que será ya en minutos.

for x in range(2):
    # por ser sólo dos valores en Vo.

    # Ao = (mAh / 1000) * C ; no se usará, vea:
    AnP = Wn/int(Vo[x]/100)  # para contrarrestar los cien que le metí antes y así aprovechar los decimales pertinentes.
    # recordemos: W = i * V.
# An = float(input("\n Inserte el Amphere en el cual trabajará el MOTOR [##.0] \n Recuerde que puede estar vinculado con su ESC= "))
# Wn = V * A # Watts (potencia) necesarios.
    An.append(int(AnP*1000))
    # añade dicho elem. a la última posición de la lista; AÑADE.
    # An.insert(x, int(AnP))  # añade, inserta en la posición x de la lista.

    Aptr = Ao / AnP  # ; A' prima igual a, "Ao" entre "A necesarios". [60*(mA*h)/mA]

    # Entonces Ap-tr será el tiempo relativo.. Ap*60 = trMin.
    trMin.append(str(Aptr * 60))
    # Ap es una CONSTANTE, NO ES Amphere; mucho menos. Entonces da.. por 60, los mins de vida.
    # aquí deberá haber un diferencial.. una especie de.

PROMtrMin = (float(trMin[0])+float(trMin[1]))/2
# convistiendolos de string a float para poder usar los decimales.. !

promAn = (An[0]+An[1])/2000
                        # Aún no sabia para qué lo iba a usar (deberá ser alguna cuenta promediada, lo cual no cuadra
                        # ya que no nosconviene hablar acerca de un A promedio puesto que no ocurrirá,
                        # ese ampheraje varía y a muy altos será en corto tiempo. NO NOS CONVIENE USAR "A promedio".
                # EDIT: sí nos conviene puesto que An[] está vinculado al Ampheraje que necesitará el motor, por el
                # hecho de que la batería posee dos voltajes.. uno cuando está cargado y otro cuando esta descargándose.

#promV = ((Vo[0]+Vo[1])/200)  # promedio voltios de los de la batería.
# ^ lo coloqué dentro del while. 

try:
    while True:
        V = (input("Coloque los Voltios a CONSUMIR [##.0] o los que suministrará constantemente!\n\t ENTER si serán los mismos que proporciona la batería. Ingrese = "))
        Vbool = bool(V)
        if (Vbool == False):
            promV = ((Vo[0] + Vo[1]) / 200) 
            V = promV  # Voltaje EN PROMEDIO..

            e = "\tSabiendo que sus voltajes fueron proporcionados EN PROMEDIO por la batería: "+str(V)+"V"
            break
        else:
            if float(V):
                V = abs(float(V))
                Vo[1] = int(V*100)
                
                promV = ((Vo[0] + Vo[1]) / 200)  # promedio voltios de los de la batería.
                
                V = promV
                e = "\t\tSabiendo que su voltaje de alimentación estaría siendo de: "+str(V)+"V"
                if V > Vo0:
                    print("\n¡Dicho valor supera al que puede otorgar la batería que tiene en uso! Inténtelo de nuevo.")
                    continue
                else:
                    break
            else:
                print("\nPor favor, ingrese únicamente en Números.")
                continue
finally:
    print("\t\nOk, suponemos que el motor estará trabajando o recibiendo, un voltaje de", str(V)+"V, constantemente.")

Wo = V * promAn  # Watts (potencia) que usted MANDARÁ a salir, de carga, DE LA BATERÍA.
                 # Voltios que proporcionará la batería. Min 18v, max 22~24v,
                 # en este caso será el promedio de no ponerlo el usuario..

td = Wo / Wn
# "Idealmente:
# Carga de la bateria: 12V x 4A = 48W
# Consumo/hora: 12V * 0.150A = 1.8W ; esta es la corriente (An[x]) que va a consumir el motor.. dando así los Wn.
# Duracion de la bateria, despreciando fugas, etc: 48W/1.8W = 26.666 Horas, o 26 Horas, 40 minutos.
# Saludos! " [extraído de http://www.todopic.com.ar/foros/index.php?topic=14382.0 el 2 de Ago, 2018. 13:30]

tdMin = td * 60  # OJO con EL NOMBRE de LA VARIABLE
print("Entonces con ello se estima que:\n\t La batería durará, despreciando fugas y demás, idealmente:",
      format(tdMin, ".2f"), "Mins.")
# OJO CON el nombre DE LA VARIABLE.

print(e, "\n\t\t..con un Amphere promedio (requeridos por el motor) de", str(format(promAn, ".2f"))+"A, \n\t\t y un "
      "tiempo promedio de descarga, en cuanto a la batería, de", format(PROMtrMin, ".2f"), "Mins.")
print("\n Hablando de unos", str(format(Wo, ".2f"))+"W por parte de la batería, y", str(format(Wn, ".2f"))+"W 'necesarios' para el avión.")

# EUPLv1.2, 2018. AEROdesignUCV - #ROADtoMexico'19 !
# Departamento General de Diseño Eléctrico.Equipo SAE Aerodesign, Universidad Central de Venezuela; Fac. Ingeniería. CCS

#TODO: queda verificar y acomodar muy bien la ecuación para hallar y sacar el tiempo de autonomía, según la batería y motor.

