/*
    You cannot have seperate functions to create and delete the moct trees,
    so this functions handles both of them.

    The other two files (createmoct.c and freemoct.c) are deprecated and are only
    for reference.

    Mex function which creates an oct tree and deletes the mocttree.
*/


#include <mex.h>
#include <stdio.h>
#include <matrix.h>
#include <string.h>
#include "mocttree.h"


/*
    ------------------- Memory Allocation code ---------------------
*/

// 1024
# define BLOCK_SIZE (1<<10)

// Allocating stuff
octnode* get_free_node(mocttree* tree){
    // If we don't have enough space in the chunk array, make some more
    if(tree->space_chunks <= tree->index_chunks){
        tree->space_chunks = (tree->space_chunks*3)/2 + 4;
        tree->memory_chunks = mxRealloc(tree->memory_chunks, tree->space_chunks*sizeof(void*));
        mexMakeMemoryPersistent(tree->memory_chunks);
    }

    // This is a new chunk
    if(tree->index_single == 0){
        // Allocating an extra octnode to align things properly
        // Otherwise its chunks of BLOCK_SIZE octnodes
        tree->memory_chunks[tree->index_chunks] = mxCalloc(BLOCK_SIZE+1, sizeof(octnode));
        mexMakeMemoryPersistent(tree->memory_chunks[tree->index_chunks]);

        uintptr_t ptr_number = (uintptr_t)tree->memory_chunks[tree->index_chunks];
        
        // Realign to 256 byte boundry
        // We align them fully so that an octnode does not straddle a page boundry
        // Importantly, this also keeps them aligned to cache line boundries
        ptr_number = (ptr_number + 255ULL)&(~(255ULL));
        tree->free_node = (octnode*)ptr_number;
    }

    // Hot path
    octnode* ret = tree->free_node++;
    tree->index_single++;

    //If we are out of space
    if (tree->index_single == BLOCK_SIZE){
        tree->index_single = 0;
        tree->index_chunks = tree->index_chunks+1;
    }

    return ret;
}

/*
    ------------------- Freeing code ---------------------
*/

// Now its used!
void free_memory(mocttree* tree){

    int chunks_to_free = tree->index_chunks;
    if (tree->index_single != 0) chunks_to_free++;

    // Free all the octnode blocks
    for(int i=0; i<chunks_to_free; i++){
        mxFree(tree->memory_chunks[i]);
    }
    // Free the space to all the octnode blocks
    mxFree(tree->memory_chunks);
    // Free the tree itself
    mxFree(tree);
}


/*
    ------------------- Construction code ---------------------
*/


/*
    Creates a node, does using the passed heap
*/
octnode* create_node(vec3 midpoint, mocttree* tree){
    // Allocate a node and fill it with zeros
    // Already zero filled (nice!)
    octnode* new_node = get_free_node(tree);

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
    mocttree* new_tree = mxCalloc(1, sizeof(mocttree));
    mexMakeMemoryPersistent(new_tree);

    new_tree->space_chunks = 0;
    new_tree->index_chunks = 0;
    new_tree->index_single = 0;
    new_tree->free_node = NULL;
    new_tree->memory_chunks = NULL;

    fix_points(&point1, &point2);
    new_tree->point1 = point1;
    new_tree->point2 = point2;
    new_tree->num_elements = 0;
    new_tree->root = create_node(vec3_midpoint(point1, point2), new_tree);
    return new_tree;
}


/*
    Insert into a node
    Requirements:
    All coordinates in point1 < node.midpoint < point2

*/
void insert_node(item new_item, octnode* node, vec3 point1, vec3 point2, mocttree* tree){
    node->num_total_elements++;
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
        node->children[which_child] = create_node(vec3_midpoint(point1, point2), tree);
    }
    
    // Insert it into the appropriate child
    // point1 and point2 were adjusted to be correct
    insert_node(new_item, node->children[which_child], point1, point2, tree);
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

    insert_node(new_item, tree->root, tree->point1, tree->point2, tree);
    return true;
}


/*
    ------------------- Entry point ---------------------
*/


/*
    This is entrypoint for this file
    in matlab it must be called as 
    createfreemoct(points, point1, point2) OR createfreemoct(treeptr)

    For the first case:
    Points is formatted as a 3xN
    [ x1 x2 x3 ... ]
    [ y1 y2 y3 ... ]
    [ z1 z2 z3 ... ]
    point1 and point2 are both arrays of 3
    doubles

    The function will return a uint64 which is a pointer to the tree

    For the second case it will destroy the tree pointed to by treeptr
    and return 0
*/


void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]){
    if (nrhs == 3){
        double* pointarray = mxGetDoubles(prhs[0]);
        size_t num_points = mxGetN(prhs[0]);

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
    } else if (nrhs == 1){

        // Freeing
        // Recast the uint64
        uint64_t* tree_pointer_int = mxGetUint64s(prhs[0])[0];
        mocttree* tree = (mocttree*)tree_pointer_int;
        free_memory(tree);

        size_t one = 1;
        mxArray* result = mxCreateUninitNumericArray(1, &one, mxUINT64_CLASS, mxREAL);
        mxGetUint64s(result)[0] = (uint64_t)0;
    } else {
        mexErrMsgIdAndTxt("Mocttree:createfreemoct:nrhs", "Bad arguments");
    }
}
