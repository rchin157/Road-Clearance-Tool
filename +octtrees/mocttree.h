/*
    Oct trees in MATLAB!
    Written by Jehanzeb Mirza
    2020-02-24
*/
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

/*
    Points for the octtree
*/
typedef struct vec3{
    double pos[3]; // Convention is x y z
} vec3;


/*
    Vector average
*/
static inline vec3 vec3_midpoint(vec3 point1, vec3 point2){
    vec3 ret;
    for (int i = 0; i < 3; i++){
        ret.pos[i] = (point1.pos[i] + point2.pos[i])/2;
    }
    return ret;
}

/*
    Vector dot
*/
static inline double vec3_dot(vec3 point1, vec3 point2){
    double dot = 0.;
    for (int i = 0; i < 3; i++){
        dot += point1.pos[i]*point2.pos[i];
    }
    return dot;
}


/*
    Each item is a struct
*/
typedef struct item{
    size_t index;
    struct vec3 point;
} item;


/*
    Designed for good cache behaviour
    Cache lines are typically 128 bytes or 64 bytes
*/
typedef struct octnode{ // 256 bytes large
    uint32_t num_elements;          // 4 bytes
    uint32_t num_total_elements;    // 4 bytes
    struct vec3 midpoint;           // 24 bytes
    struct item bucket[5];          // 5*32 = 160 bytes
    struct octnode* children[8];    // 8*8 = 64 bytes
} octnode;


/*
    Main structure of the C function
*/
typedef struct mocttree{
    // Corners
    // for every element point1 <= point2
    struct vec3 point1;
    struct vec3 point2;
    struct octnode* root; // Always has a root
    size_t num_elements;
    
    // For memory
    size_t space_chunks; // Points to last avaible
    size_t index_chunks; // Points to the last used
    size_t index_single;
    octnode* free_node;
    void** memory_chunks; 
    // Don't index into memory chunks, thats why I left it void**
    // Its technically a pointer to an array of pointers to octnodes (sorta), but the alignments
    // are off so indexing will corrupt data.
} mocttree;
