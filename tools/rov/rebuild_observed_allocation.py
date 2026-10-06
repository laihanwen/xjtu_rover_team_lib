from pathlib import Path
p=.416699;q=.807913
positions=[(.18,.12,.058),(.18,-.12,.058),(.18,-.12,-.058),(.18,.12,-.058),(-.18,.12,.058),(-.18,-.12,.058),(-.18,-.12,-.058),(-.18,.12,-.058)]
directions=[(p,p,-q),(p,-p,-q),(p,-p,q),(p,p,q),(-p,p,-q),(-p,-p,-q),(-p,-p,q),(-p,p,q)]
cols=[]
for (x,y,z),(a,b,c) in zip(positions,directions):cols.append([a,b,c,y*c-z*b,z*a-x*c,x*b-y*a])
B=[list(row) for row in zip(*cols)]
G=[[sum(B[i][k]*B[j][k] for k in range(8)) for j in range(6)] for i in range(6)]
aug=[row+[float(i==j) for j in range(6)] for i,row in enumerate(G)]
for i in range(6):
    pivot=max(range(i,6),key=lambda j:abs(aug[j][i]));aug[i],aug[pivot]=aug[pivot],aug[i]
    assert abs(aug[i][i])>1e-9,'geometry rank deficient'
    v=aug[i][i];aug[i]=[x/v for x in aug[i]]
    for j in range(6):
        if j!=i:
            v=aug[j][i];aug[j]=[a-v*b for a,b in zip(aug[j],aug[i])]
inv=[row[6:] for row in aug]
A=[[sum(B[k][i]*inv[k][j] for k in range(6)) for j in range(6)] for i in range(8)]
gains=[.85,.85,.62,.85,.47,.85]
for j in range(6):
    scale=gains[j]/max(abs(row[j]) for row in A)
    for row in A:row[j]*=scale
fmt=lambda rows:'{\n'+',\n'.join('    { '+', '.join(f'{v:.9f}f' for v in row)+' }' for row in rows)+'\n}'
header='''/* Positive actual PWM body force = negative of operator-observed jet.
 * Signs measured 2026-10-06; angles/positions retain provisional geometry.
 * Body +X front, +Y left, +Z up. No separate polarity inversion.
 * Axis order Fx,Fy,Fz,Mx,My,Mz; normalized pseudoinverse of [d;r cross d]. */
#ifndef AUV_THRUSTER_ALLOCATION_H
#define AUV_THRUSTER_ALLOCATION_H
'''
header+='static const float vector_allocation_matrix[8][6] = '+fmt(A)+';\n'
header+='#ifdef AUV_ALLOCATION_TEST\nstatic const float auv_physical_wrench[6][8] = '+fmt(B)+';\n#endif\n#endif\n'
Path('firmware/stm32/MDK-ARM/AuvThrusterAllocation.h').write_text(header)
f=Path('firmware/stm32/MDK-ARM/Move.c');s=f.read_text(encoding='utf-8');start=s.index('/*\n * Body frame:');end=s.index('static void VectorAllocate',start);s=s[:start]+'#include "AuvThrusterAllocation.h"\n\n'+s[end:];f.write_text(s,encoding='utf-8')
f=Path('firmware/stm32/MDK-ARM/Move.h');s=f.read_text(encoding='utf-8');start=s.index('/*\n * Motor polarity');end=s.index('/* Paper-style',start);s=s[:start]+'/* Allocation uses actual positive PWM directions measured on all channels. */\n'+''.join(f'#define Motor_{i}Polarity 1\n' for i in range(1,9))+'\n'+s[end:];f.write_text(s,encoding='utf-8')
print('matrix reconstructed, full rank six')
