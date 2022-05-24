/*
    Query Functions used for querying
*/

#pragma once
#include "mocttree.h"


/*
    Abstract plane class
    Planes are defined as
    ax + by + cz = d
*/
typedef struct plane3{
    struct vec3 norm;
    double dval;
} plane3;


/*
    Abstract region
    A point <x,y,z> is considered "inside" iff
    ax + by + cz >= d for all planes
*/
typedef struct constraint{
    size_t num_planes;
    struct plane3 planes[];
} constraint;


/*
    Checks if a point is inside a constraint
*/
bool satisfies(constraint* cons, vec3 point){
    for (int i = 0; i < cons->num_planes; i++){
        if (vec3_dot(cons->planes[i].norm, point) < cons->planes[i].dval) return false;
    }
    return true;
}


/*
    Check if the region defined by a cube with corners
    point1 and point2 has any satisfying space

    This is definitely false if for any of the planes all corners of the cube
    do not satisfy.

    So our result is a definitely false, probably true.

    The complete solution (definitely false, definitely true) is a linear programming question.
    (The cube itself can be represented as a set of 6 constraints, you must check
    if the system has any solutions). Its probably slower than adding a few extra layers of checks

    False negatives are excedingly rare as the cubes become smaller, which they do so exponentially.
    So it does not matter, and we write ourselves a fast O(n) Algorithmn instead.
*/
bool cube_satisfies(constraint* cons, vec3 point1, vec3 point2){
    vec3 temp;
    for (int p = 0; p < cons->num_planes; p++){
        int i;
        for (i = 0; i < 8; i++){
            // Creates every possible pairing of point1 and point2
            temp.pos[0] = i&1 ? point1.pos[0] : point2.pos[0];
            temp.pos[1] = i&2 ? point1.pos[1] : point2.pos[1];
            temp.pos[2] = i&4 ? point1.pos[2] : point2.pos[2];
            if (vec3_dot(cons->planes[p].norm, temp) >= cons->planes[p].dval) break; // If that pairing satisfies
        }
        if (i == 8) return false;
    }
    return true;
}


/*
    True if all points within a cube fully satisfies a constraint, this is for an efficient
    search.

    Its true if all corners of the cube fully satisfy (shape is convex so all points inside
    must too then).
*/
bool cube_fully_satisfies(constraint* cons, vec3 point1, vec3 point2){
    vec3 temp;
    for (int i = 0; i < 8; i++){
        // Creates every possible pairing of point1 and point2
        temp.pos[0] = i&1 ? point1.pos[0] : point2.pos[0];
        temp.pos[1] = i&2 ? point1.pos[1] : point2.pos[1];
        temp.pos[2] = i&4 ? point1.pos[2] : point2.pos[2];
        if (!satisfies(cons, temp)) return false;
    }
    return true;
}
