import math

print('Hola, sean bienvenidos/as al programa test de baterías para Radio Control (R/C),\n'
      'escrito con lenguaje Python 3.6.1 en licencia \033[1mEUPL-1.2\x1B[3m elaborado por, y para el\n'
      'equipo Aerodesign UCV. 2018; a través del compilador Pycharm, y el online Repl.it,\n'
      'con repositorio en GitHub, bajo el NOMBRE: "EnergySupplyRC".', '¡En la versión 1.0.1! \nLes da la cordial bienvenida,')
print("\t\t\t\t\t\t\t\t\t\t\t",
      "~V. Fabian T. Mendes Castillo. \n\t\t\t\t\t\t\t\t\t\t\t"
      "\x1B[3mAerodesignUCV's Electrical E. Student,",
      "\n\t\t\t\t\t\t\t\t\t\t\tG. Departament of Electrical Design Chief.\x1B[3m")
print('''\033[1mP.S.\x1B[3m:''',
      '\033[1mEstos resultados son idealmente prácticos, calculados bajo condiciones ideales.\033[3m\n')

try:
    while True:
        mAh = input("Ingrese el almacenamiento de la batería [mAh]: ")
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
            if int(nS) != False and (
                    int(nS) >= 1 or int(nS) <= 20):  # puede simplificarse solo colocando: 'if float(var):'
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
                
                if C in range(10,101):
                    C = int(C)
                    C = abs(C)
                    break
                else:
                    print("\nPor favor, ingrese únicamente en Números NATURALES ( > 09 ) [de 25~70C].")
                    continue
            else:
                print("\nPor favor, ingrese únicamente en Números NATURALES ( > 09 ) [de 25~70C].")
                continue
        else:
            print("\nNO DEBE dejar en blanco si desea continuar.")
            continue
    i = 1
    while i != 0:
        Wn = input(
            "\n\t Inserte los Watts (Potencia nominal, o máx) del MOTOR,"  # cuidado acá, no se pone coma!
            "\n\t\t\t\t\to los que querrá suministrar-le [####W]= ")       # ya que tratamos no un INPUT.
        if Wn:
            if float(Wn):  # se omite el '!= False' porque se considera y asume que esto signinica que "si es verdad.."
                Wn = int(abs(float(Wn)))
                if Wn >= 99999:
                    print("\nVERIFIQUE los Watts escritos, es muy grande.")
                    i = 1
                else:
                    if Wn in range(100):
                        # hp = float(Wn)/745.7 # está en el finally.
                        print("\nVERIFIQUE los Watts escritos, es muy pequeño (> 100W).")
                        i = 1
                    elif Wn >= 1000:
                        print("Registrado. Tenga en cuenta que, para competencias SAE Aerodesign,",
                              "\nuna de las restricciones es que lleguen  < 1000 W al motor. Continuemos..")
                        i = 0
                    else:
                        i = 0
            else:
                print("Por favor, ingrese únicamente en Números.")
                i = 1
        else:
            print("NO DEBE dejar en blanco si desea continuar.")
            i = 1

finally:
    print(
        "\n\t Batería de", str(nS) + " S (celdas) y", str(mAh), "mAh. Y una capacidad de descarga de", str(C) + "C.",
        "\n\t Motor brushless con", str(Wn),
        "Watts necesarios o máx. A lo que es 'igual' o menor que:", str(format(float(Wn) / 745.7, ".2f")) + "HP (aprox.)")

Ao = (mAh / 1000) * C  # se comentó porque este Ao es de la batería mismo. Yeah!! Y con los que buscaremos el ESC.
# ; La Capacidad, recordemos que hay dos:
# continua, y de explosición (por unos mSeg, máx POWER).
# y EN REALIDAD ES: # En una hora puede dar ( )mA a una capacidad de descarga de ( )C = Ao
# Listo.

Ab = (Ao / C)  # Ab son los ampere de la batería. Los cuales serán usados para medir el T. Recordemos que esta dividido por 1h.

print("\t\tAmphere que PODRÍA llegar a pasar por el ESC: Ao = ",
      str(format(Ao, ".2f")) + "A, según los datos de la batería. \n\tTenga cidado en su selección del ESC;"
      "\n\t\t\t se recomienda que el máximo amperaje de éste sea mayor a " + str(format(Ao, ".2f")) + "A, o casi el doble.\n")
Mult = 1000  # Multiplicador para poder aprovechar los decimales, ya que se perderán al convertirse en int().

Vo1 = nS * 4.1867  # 'Vo[1]' Voltaje de la batería (Vo) iniciales (t = 0), cargado!
Vo0 = nS * 3.6544  # 'Vo[0]' Voltaje de la batería (Vo) a punto de descargarse ( = 1)

Vo = [int(Vo1 * Mult), int(Vo0 * Mult)]  # máx. Voltios en una batería; 3.7v min y 4.2 máx, o inicial.
A = []  # Ampheres que deberan poder estar pasando (requeridos..) por el ESC según (..por:) el MOTOR!

# promV = ((Vo[0]+Vo[1])/200)  # promedio voltios de los de la batería.
# ^ lo coloqué dentro del while.

try:
    while True:
        V = (input("Coloque los Voltios a CONSUMIR [##.0] o los que suministrará constantemente!"
            "\nENTER si serán los mismos que proporciona la batería, escriba '01' para STORAGE MODE. Ingrese = "))
        Vbool = bool(V)
        if not Vbool:
            promV = ((Vo[0] + Vo[1]) / (2 * Mult))
            V = promV  # Voltaje EN PROMEDIO..

            e = "\tSabiendo que sus voltajes proporcionados por la 'batería', serán EN PROMEDIO, de: " + str(
                format(V, ".2f")) + "V"
            break
        elif int(V) == 1:

            Vo[0] = int(float(Vo[1]) + (float(Vo[0]) - float(Vo[1])) / 2)

            promV = (float(Vo[0] + Vo[1]) / (2 * Mult))
            V = promV  # Voltaje EN PROMEDIO..

            e = "\tSabiendo que sus voltajes fueron proporcionados por una batería con configuración STORAGE MODE: " + str(
                format(Vo[0]/Mult, ".2f")) + "V"
            break
        else:
            if float(V):
                V = abs(float(V))
                Vp = int(V * Mult)
                if Vp > Vo[0]:

                    print("\n¡Dicho valor SUPERA al que puede otorgar la batería que tiene en uso! Inténtelo de nuevo.",
                          "[ " + str(format(Vo[0]/Mult, ".2f")) + "~" + str(format(Vo[1]/Mult, ".2f")) + " V]")
                    continue
                elif Vp < Vo[1]:

                    print("\n¡Dicho valor ES MENOR al que la batería que tiene en uso puede otorgar! Inténtelo de nuevo.",
                          "[ " + str(format(Vo[0]/Mult, ".2f")) + "~" + str(format(Vo[1]/Mult, ".2f")) + " V]")
                    continue
                else:

                    Vo[0] = Vp  # inicialmente, por si esta ya 'usada' parecido a cargada en config. STORAGE!

                    promV = ((Vo[0] + Vo[1]) / (2 * Mult))  # promedio voltios de los de la batería.

                    V = promV
                    e = "\t\tSabiendo que su voltaje de alimentación estaría siendo, en promedio, de: " + str(
                        format(V, ".2f")) + "V"
                    break
            else:
                print("\nPor favor, ingrese únicamente en Números.\t Si su batería estuvo en STORAGE MODE, escriba '01'")
                continue
finally:
    print("\t\nOk, suponemos que el motor estará trabajando o recibiendo, un voltaje de",
          str(format(Vo[0]/Mult, ".2f")) + "V, inicialmente.")

Tmin = []  # tiempo relativo que será ya en minutos.

for x in range(2):
    # por ser sólo dos valores en Vo.

    # Ao = (mAh / 1000) * C ; no se usará, vea:
    Ap = Wn / int(Vo[x] / Mult)  # Mult para contrarrestar los cien que le metí antes y así aprovechar los decimales pertinentes. Prima
    # recordemos: W = i * V. Y de ello, con los Watts del motor y los Voltios de la batería damos con el A para nuestro T.
    # T = Ab / (A * C) ; C es inversamente proporcional a la corriente (en este caso, Ab) de la batería para su tiempo de descarga.

    A.append(int(Ap * Mult))
    # se multiplica por Mult para luego lograr usar los decimales, puesto que las listas solo sportan str() e int().
    # añade dicho elem. a la última posición de la lista; AÑADE.
    # An.insert(x, int(AnP))  # añade, inserta en la posición x de la lista.

    Aptr = (Ab*Mult) / Ap  # ; A' prima igual a, "Ao" entre "A necesarios". [para luego, 60*(mA*h)/mA]

    # Entonces Ap-tr será el 'tiempo relativo'.. Ap*60 = Tmin. Que introduciremos consiguientemente:
    Tmin.append(str(Aptr * 60))
    # Ap es una CONSTANTE, NO ES Amphere; mucho menos. Entonces da.. por 60, los mins de vida.
    # aquí deberá haber un diferencial.. una especie de.

promTmin = ((float(Tmin[0]) + float(Tmin[1])) / (2 * Mult))
# convistiendolos de string a float para poder usar los decimales.. !

promA = (A[0] + A[1]) / (2*Mult)
# Aún no sabia para qué lo iba a usar (deberá ser alguna cuenta promediada, lo cual no cuadra
# ya que no nos conviene hablar acerca de un A promedio puesto que no ocurrirá,
# ese amperaje varía y a muy altos A será en corto tiempo. NO NOS CONVIENE USAR "A promedio".
# EDIT: sí nos conviene puesto que A[] está vinculado al Ampheraje que necesitará el motor, por el
# hecho de que la batería posee dos voltajes.. uno cuando está cargado y otro cuando está descargándose.

Wp = V * promA  # Watts (potencia) que usted MANDARÁ a salir, de carga, DE LA BATERÍA.
# ..en promedio!
# Voltios que proporcionará la batería. Min 18v, max 22~24v,
# en este caso será el promedio de no ponerlo el usuario..
# esos V están promediados.
Wo = []
# td = Wp / Wn  # esto no fue cierto... COMÉNTESE! Esta función no servirá de nada.
# "Idealmente:
# Carga de la bateria: 12V x 4A = 48W
# Consumo/hora: 12V * 0.150A = 1.8W ; esta es la corriente (An[x]) que va a consumir el motor.. dando así los Wn.
# Duracion de la bateria, despreciando fugas, etc: 48W/1.8W = 26.666 Horas, o 26 Horas, 40 minutos.
# Saludos! " [extraído de http://www.todopic.com.ar/foros/index.php?topic=14382.0 el 2 de Ago, 2018. 13:30]
# tdMin = td * 60  # OJO con EL NOMBRE de LA VARIABLE
#  ^  ya se hizo más arriba, como 'Tmin'

for x in range(2):
    W = float(Vo[x] * A[x]) / pow(Mult, 2)  # RECORDAR que ambos estaban Mult. por 1000.. (Mult = 10^3)
    Wo.append(str(format(W, ".2f")))


TminImpri = float(Tmin[1]) / Mult
# EROR Resuelto. Había que convertir el valor que estaba en la lista 'Tmin[]' porque era str()...

print("Entonces con ello se estima que:\n\t\t La batería durará, despreciando fugas y demás, idealmente: "
      + str(format(TminImpri, ".2f")) + " Mins. MÁX!")
# OJO CON el nombre DE LA VARIABLE.

print(e, "\n\t\t..con un Amphere promedio (requeridos por el motor) de",
      str(format(promA, ".2f")) + "A, \n\t\t y un tiempo promedio de descarga, en cuanto a la batería, de",
      format(promTmin, ".2f"), "Mins.")
print("\n Hablando de unos", str(format(Wp, ".2f")) + "W en promedio por parte de la batería, y\n\t",
      Wo[1] + "~" + Wo[0] + "W 'necesarios' o máx. hacia el avión, es decir, \nson los que podrían pasar por el Limitador de potencia en algún momento dado!")

# EnergySupplyRC v1.0.1
# EUPLv1.2, 2018. AEROdesignUCV - #ROADtoMexico'19 !
# Departamento General de Diseño Eléctrico. Equipo SAE Aerodesign, Universidad Central de Venezuela; Fac. Ingeniería. CCS

# ______________________________________________

# TODO energysupply.py (v1.1.0) traducido al inglés, y posiblemente portugués.
# Todo: energysupply + ".py" Next-version (2.0), con las eficiencias y las temperaturas!

# TODO energysupplyRC Version 3.0, ayudarse con mathlab u otro programa! Quizas ya no sea con ".py"
