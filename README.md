# TreeEngine

A procedural tree mesh generator and physics simulator

## Description

This is a WebGPU application that uses L-Systems to generate realistic trees of various types, and then uses compute shaders to simulate physics such as wind, gravity, collisions, and other factors affecting the movement of trees. 

## Screenshots

![alt text](https://jackblazes.net/assets/graphics/TreeEngine/tree_thumbnail.png)

## Getting Started

### Dependencies

* Only runs on Chrome and Safari, as far as I'm aware

### Building

* For local use, you can run these commands:

- cmake . -B build

- cmake --build build


This will produce an executable (build/Debug/App.exe on Windows). 
It is possible to build an Emscripten web build.

### Running

Simply run the executable (build/Debug/App.exe on Windows)

## Authors

Jack Blazes

https://jackblazes.net/

blazec@alum.mit.edu

## License

This project is licensed under the [NAME HERE] License - see the LICENSE.md file for details