[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-2e0aaae1b6195c2367325f4f02e2d04e9abb55f0b24a779b69b11b9e10269abc.svg)](https://classroom.github.com/online_ide?assignment_repo_id=16775483&assignment_repo_type=AssignmentRepo)
# Ejemplo base de Jack en C++

Este ejemplo construye una aplicación muy sencilla de "pass-through"
usando Jack, como punto de partida para los proyectos y tareas del
curso.  Usa además Qt para crear una interfaz gráfica sencilla como ejemplo.

Esta versión permite recibir una lista de archivos
.wav, que se ejecutan uno tras otro, reemplazando la entrada de
micrófono en tanto hayan datos de los archivos disponibles.  Una vez
que todos los archivos terminan de ejecutarse, regresa al modo
"pass-through".

En esta versión para el proyecto 2, el pass-through se realiza de una
forma poco eficiente en el dominio de la frecuencia, solo como
ejemplo de cómo utilizar la clase freq_filter (ya usada en la tarea
4).  Provee además mecanismos para cambiar el volumen y para
interrumpir la ejecución de archivos de audio, de modo que desde la 
GUI se puedan controlar, a manera de ejemplo.

## Dependencias

Requiere C++ en su estándar del 2020 (g++ 12, clang 14).

En derivados de debian (ubuntu, etc):

    sudo apt install jackd2 libjack-jackd2-dev qjackctl build-essential meson ninja-build jack-tools libsndfile1-dev libsndfile1 libboost-all-dev libfftw3-dev libfftw3-bin libfftw3-single3 qtcreator libqt5widgets5 libqt5gui5 libqt5core5a libqt5designer5 libqt5designercomponents5 qt5-qmake qtbase5-dev qtbase5-dev-tools libqcustomplot-dev
     
Jack requiere que su usuario pertenezca al grupo audio, o de otro modo
no tendrá privilegios para el procesamiento demandante en tiempo
real...

     sudo usermod -aG audio <su usuario>

## Construcción

En este ejemplo hay dos posibilidades de construcción: meson y qmake. 

QMake es el método nativo de Qt, pero no es versatil en cuanto a otras
bibliotecas se refiere.  Meson es útil con cosas generales, pero no es
muy bueno con los procesos de compilación especializados de Qt, por lo
que cada vez que se cambia la GUI puede ser necesario tener que
reconstruir el directorio completo.

Para construir los ejemplos la primera vez utilice

     meson setup builddir
     cd builddir
     ninja


Si requiere reconstruir todo (por ejemplo, porque cambió la GUI), utilice

     meson setup --wipe builddir
     cd builddir
     ninja

o si solo requiere reconfigurar por haber agregado otro archivo:

    meson --reconfigure builddir

Por otro lado, si desea utilizar qmake, entonces utilícelo así:

     mkdir buildqt
     cd buildqt
     qmake ../dsp_proy2.pro

Eso construye un archivo Makefile en el directorio buildqt.  Para 
construir la aplicación basta con llamar a

     make

Por supuesto, también puede utilizar qtcreator y manejar la 
construcción del ejecutable desde allí.


## Latencia y tamaño de bloque

Para reducir la latencia por medio del tamaño del "periodo" (esto es,
el número de "frames" que cada ciclo de procesamiento recibe, en
QjackCtl, en Settings, se indica en Frames/Period.  Eso es un
parámetro del servidor de Jack y no lo puede controlar la aplicación
como tal.
