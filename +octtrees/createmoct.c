/*
    Mex function which creates an oct tree!

    DEPRECATED - SEE CREATEFREE
*/
#include <mex.h>
#include <matrix.h>
#include <string.h>
#include "mocttree.h"


/*
    Creates a node, does using the passed heap
*/
octnode* create_node(vec3 midpoint, mi_heap_t* heap){
    
    // Allocate a node and fill it with zeros
    octnode* new_node = mi_heap_malloc_small(heap, sizeof(octnode));
    memset(new_node, 0, sizeof(octnode));

    new_node->midpoint = midpoint;
    return new_node;
}


/*
    Ensures every coordinate in point1 is < point2
*/
void fix_points(vec3* point1, vec3* point2){
    double temp;
    for (int i = 0; i < 3; i++){
        if(point2->pos[i] < point1->pos[i]){
            temp = point2->pos[i];
            point2->pos[i] = point1->pos[i];
            point1->pos[i] = temp;
        }
    }
}


/*
    Creates the base of the tree
*/
mocttree* create_tree(vec3 point1, vec3 point2){
    // Note: No special requirements on point1 or point2

    // The tree itself is stored under MATLAB's memory manager
    mocttree* new_tree = mxMalloc(sizeof(mocttree));
    mexMakeMemoryPersistent(new_tree);

    // Construct a heap for the tree
    new_tree->heap = mi_heap_new();

    fix_points(&point1, &point2);
    new_tree->point1 = point1;
    new_tree->point2 = point2;
    new_tree->num_elements = 0;
    new_tree->root = create_node(vec3_midpoint(point1, point2), new_tree->heap);

    return new_tree;
}


/*
    Insert into a node
    Requirements:
    All coordinates in point1 < node.midpoint < point2

*/
void insert_node(item new_item, octnode* node, vec3 point1, vec3 point2, mi_heap_t* heap){
    if (node->num_elements < 5){
        node->bucket[node->num_elements++] = new_item;
        return;
    }

    int which_child = 0;
    // X Y Z indexing
    // if X > midpoint X then +1
    // if Y > midpoint Y then +2
    // if Z > midpoint Z then +4
    for (int i = 0; i < 3; i++){
        if (new_item.point.pos[i] > node->midpoint.pos[i]){
            which_child += (1<<i);
            point1.pos[i] = node->midpoint.pos[i];
        } else {
            point2.pos[i] = node->midpoint.pos[i];
        }
    }

    // Create the child should it not exist
    if (node->children[which_child] == NULL){
        node->children[which_child] = create_node(vec3_midpoint(point1, point2), heap);
    }
    
    // Insert it into the appropriate child
    // point1 and point2 were adjusted to be correct
    insert_node(new_item, node->children[which_child], point1, point2, heap);
}


/*
    Insert into a tree
*/
bool insert_tree(mocttree* tree, vec3 point){

    item new_item;
    new_item.index = ++(tree->num_elements);
    new_item.point = point;

    // Ensure its inside the tree
    for (int i = 0; i < 3; i++){
        if(point.pos[i] < tree->point1.pos[i] || point.pos[i] > tree->point2.pos[i]) return false;
    } // Note, With the current implementation (static trees) this is always true

    insert_node(new_item, tree->root, tree->point1, tree->point2, tree->heap);
    return true;
}


/*
    This is entrypoint for this file
    in matlab it must be called as 
    createmoct(points, point1, point2)
    where points is formatted as a 3xN
    [ x1 x2 x3 ... ]
    [ y1 y2 y3 ... ]
    [ z1 z2 z3 ... ]
    point1 and point2 are both arrays of 3
    doubles
*/
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]){
    if (nrhs != 3){
        mexErrMsgIdAndTxt("Mocttree:createMoct:nrhs", "Bad arguments");
    }

    double* pointarray = mxGetDoubles(prhs[0]);
    size_t num_points = mxGetDimensions(prhs[0])[1];

    double* point1arr = mxGetDoubles(prhs[1]);
    double* point2arr = mxGetDoubles(prhs[2]);

    vec3 point1;
    vec3 point2;

    // This expands the octtree by 10% each way
    for (int i = 0; i < 3; i++){
        point1.pos[i] = 1.1*point1arr[i] - 0.1*point2arr[i];
        point2.pos[i] = 1.1*point2arr[i] - 0.1*point1arr[i];
    }

    mocttree* tree = create_tree(point1, point2);
    vec3 new_point;

    for (int i = 0; i<num_points; i++){
        new_point.pos[0] = pointarray[3*i+0];
        new_point.pos[1] = pointarray[3*i+1];
        new_point.pos[2] = pointarray[3*i+2];    
        insert_tree(tree, new_point);
    }

    size_t one = 1;
    mxArray* result = mxCreateUninitNumericArray(1, &one, mxUINT64_CLASS, mxREAL);
    mxGetUint64s(result)[0] = (uint64_t)tree;

    plhs[0] = result;
}
