Welcome to 42, the mostly harmless spacecraft simulation tool.

See the overview, "42 Overview.pdf", in the Docs folder. Also recommended:
Nomenclature.pdf
POV Menu.pdf
Key Bindings.txt
FSW Models.pdf
Flight Regimes.pdf

If you're installing on Windows, see the file "Install-msys.txt" in the Docs folder.

The compiler will attempt to detect what platform you're on (Linux, OSX, or Msys), but its success rate isn't great.  If you have errors on the first compile or run, try editing your Makefile to manually set your 42PLATFORM.  

The default folder for inputs and outputs is "InOut".  Look there for sample input files.  "Inp_Sim.txt" is the top-level input file.

The input/output folder may be changed for a sim run by running 42 with an argument.  For example, enter this at the command prompt:
42 Demo


Common Problems:
1)  42 expects the input files to be plain vanilla text files.  If your text editor adds formatting, makes straight quotes into smart quotes, etc, 42 may get confused.  The most common symptom is generating the "Bogus input in DecodeString" error.
2)  Also text-related, 42 is very simple-minded about reading the input files.  Adding extra lines, or accidentally deleting a line, or swapping lines, will throw things out of synch.  Again, the most common symptom is the "Bogus input in DecodeString" error.  Use your debugger to trace back where the error was generating.  (The actual mistake may be at that line, or may be somewhere upstream.)


Description of Folder Contents:

Database: Contains data files to be stored for input / output from simulation.

Docs: Contains the supporting documentation and references for the files used in the 42 simulation.

Include: Contains the header files for the source code. 

InOut: Stores the command script files used in simulation. This includes the input commands for the initial conditions of the simulation, the simulation parameters, the orbit parameters, the spacecraft parameters, and other input files.

Kit: Contains low level functions used in the source code, for example math or vector functions. 

Object: Populates with object files once the source code is compiled. 

References: Contains develepmental references included by the creators of 42 and licensing information.
	-The Demo folder contains a fully functional demo simulation which is helpful in understanding the capability of the simulation.
		-To run the demo, move it into your main 42 folder, rename it InOut, and compile the code (open Msys, change directory to 42 folder, make clean, make, 42).

Source: Contains the C source code. This code is what defines the processes of the simulation. If you wish to adjust the behavior of any components, such as actuators or the control algorithm, you would modify it in this code.

World: Contains environmental models used in the simulation GUI.

