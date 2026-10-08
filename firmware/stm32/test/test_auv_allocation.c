#include "AuvThrusterAllocation.h"
#include <assert.h>
#include <math.h>
int main(void) {
    const float r[8][3]={{.18f,.12f,.058f},{.18f,-.12f,.058f},{.18f,-.12f,-.058f},{.18f,.12f,-.058f},{-.18f,.12f,.058f},{-.18f,-.12f,.058f},{-.18f,-.12f,-.058f},{-.18f,.12f,-.058f}};
    /* Independent observed jet signs, actual PWM +50; force is opposite. */
    const int jet[8][3]={{-1,-1,1},{-1,1,1},{-1,1,-1},{-1,-1,-1},{1,-1,1},{1,1,1},{1,1,-1},{1,-1,-1}};
    float B[6][8], gain[6];
    for(int i=0;i<8;i++) {
        float d[3]={-.416699f*jet[i][0],-.416699f*jet[i][1],-.807913f*jet[i][2]};
        for(int j=0;j<3;j++)B[j][i]=d[j];
        B[3][i]=r[i][1]*d[2]-r[i][2]*d[1];
        B[4][i]=r[i][2]*d[0]-r[i][0]*d[2];
        B[5][i]=r[i][0]*d[1]-r[i][1]*d[0];
    }
    for(int axis=0;axis<6;axis++)for(int out=0;out<6;out++) {
        float total=0;for(int i=0;i<8;i++)total+=B[out][i]*vector_allocation_matrix[i][axis];
        if(axis==out){assert(total>0.01f);gain[axis]=total;}
        else assert(fabsf(total)<0.00001f);
    }
    /* Mixed translation must still have zero rotation and correct force ratio. */
    const float cmd[6]={.7f,.4f,-.3f,.12f,-.2f,.1f};
    for(int out=0;out<6;out++) {
        float total=0;
        for(int i=0;i<8;i++){float motor=0;for(int a=0;a<6;a++)motor+=vector_allocation_matrix[i][a]*cmd[a];total+=B[out][i]*motor;}
        assert(fabsf(total-gain[out]*cmd[out])<.00001f);
    }
    return 0;
}
