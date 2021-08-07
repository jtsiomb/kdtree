#include <stdio.h>
#include "kdtree.h"


int main(void) {
    void* tree = kd_create(2);
    void* res;
    double data[] = {
        2, 3,
        5, 4,
        9, 6,
        4, 7,
        8, 1,
        7, 2
    };
    double pos[2] = {9, 2};
    kd_insert(tree, &data[0], NULL);
    kd_insert(tree, &data[2], NULL);
    kd_insert(tree, &data[4], NULL);
    kd_insert(tree, &data[6], NULL);
    kd_insert(tree, &data[8], NULL);
    kd_insert(tree, &data[10], NULL);

    res = kd_nearest_n(tree, pos, 3);
    while(!kd_res_end(res)) {
        kd_res_item(res, &data[0]);
        printf("%f %f %f\n", data[0], data[1], kd_res_dist(res));
        kd_res_next(res);
    }
    kd_res_free(res);
	kd_free(tree);
    return 0;
}
