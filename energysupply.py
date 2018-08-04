import math

mAh = int(input("Ingrese el almacenamiento de la batería [mAh]: "))
  # los miliAmphere, por hora, (mAh) de la batería!
C  = int(input("Ingrese la Capacidad de la bateria [25~70C]= "))
 # Capacidad de la batería, o de explosicón según necesite el usuario (la máx, y durante pocos segundos. Ver especificaciones de la batería!


Ao = (mAh/1000)*C # será el Amphere que suministrará según la Capacidad.
Vo = float(input("Ingrese los Voltios DE LA batería [##.0])= "))
  # Voltios que proporcionará la batería. Min 18v, max 22~24v
Wo = Vo * Ao  # Watts (potencia) iniciales, de carga, de la batería.
V = (input("\n Ingrese los Voltios a CONSUMIR [##.0] \n ENTER si serán los mismos que INGRESÓ anteriormente= "))
Vbool = bool(V)
if (Vbool == False):
  V = Vo
else:
  V = float(V)

Wn = float(input("\n Inserte los Watts (Potencia nominal, o máx) que necesitará suministrar al MOTOR [##]= "))

A = Wn/V
  #A = float(input("\n Inserte el Amphere en el cual trabajará el MOTOR [##.0] \n Recuerde que puede estar vinculado con su ESC= "))
  #Wn = V * A # Watts (potencia) necesarios.
td = Wo/Wn 
  # "Idealmente:
  # Carga de la bateria: 12V x 4A = 48W
  # Consumo/hora: 12V * 0.150A = 1.8W
  # Duracion de la bateria, despreciando fugas, etc: 48W/1.8W = 26.666 Horas, o 26 Horas, 40 minutos.
  # Saludos! " [extraído de http://www.todopic.com.ar/foros/index.php?topic=14382.0 el 2 de Ago, 2018. 13:30]


tdMin = td*60 # OJO con EL NOMBRE de LA VARIABLE
print("\nLa batería durará, despreciando fugas y demás, idealmente: ", tdMin,"Mins")
  #OJO CON el nombre DE LA VARIABLE.

print(" Sabiendo que sus voltajes fueron, Vo=", Vo,"y V=", V,"y Amphere: Ao=", Ao, "y A (motor)=", A)
print("\n Hablando de unos", Wo,"Watts por parte de la batería, y W=", Wn, "necesarios del avión")
  # print(Wo," y ",V) # Así se imprime. Sin ERRORes


# EUPL-1.2, by Aerodesign UCV. 2018

#TODO ARREGLAR el suministro de Watts por parte de la batería. Ya que recordemos no deben llegarle los 1000W,
  #TODO además, una cosa es cuanto 'pedirá' el motor, y otra cuanto se le suministrará. Aunque aquí pensandolo deberia es cambiarse la interpretación del texto..
  #TODO ya que, supongo, la batería tendrá unos watts, pero le daremos menos al motor, por lo que debería durar más! Por ende se le escribió a unos equipos de competencia para su asesoría,
  #ya que es necesario saber si es normal que una batería dure poco o mucho, o mejor dicho: en promedio cuánto dura una batería.
  #TODO adicionalmente HAY QUE probar el calculo de la Potencia Máx del motor en función a su peso.
#TODO: crear directorio que se coloque el número de celdas en vez de los mAh, o mejor!
  #Si se coloca de entrada una sola cifra, menor que 10 celdas (S) y de allí seleccione el voltaje.
#TODO Tambien HABILITAR el poder colocar una cierta carga de la batería, es decir, a su máxima carga (4.2v) y la minima (de 3.7v)
