# LO41 Drone Sim

A LO41 project about delivery drones.

## How to build and run

To build the project you can use the make command with the provided Makefile.

The usage is as follow:

- `$ make`, for a classical build.
- `$ make run`, to build and run.
- `$ make debug`, for a debug build. It also runs directly the program.
- `$ make clean`, to clean files produced by compilation.

The binary is to be found in the `./bin` folder.

To run manually the software with default parameters, use the command `$ ./bin/LO41_drone_sim.elf`.

## Command usage

The software can be run with the following arguments:

    --speed <speed>`   to specify the minimum duration in milliseconds of each simulation tick.
    --file <csv_file>` to specify the simulation csv file to use.

## No memory leaks

We checked the software for memory leaks with Valgrind. Here is the heap summary:

    ==7391== HEAP SUMMARY:
    ==7391==     in use at exit: 0 bytes in 0 blocks
    ==7391==   total heap usage: 706 allocs, 706 frees, 111,821 bytes allocated
    ==7391==
    ==7391== All heap blocks were freed -- no leaks are possible

