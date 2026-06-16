# grid = [
#     [-1, -1, -1, -1],
#     [-1,  0,  0, -1],
#     [-1,100, 0, -1],
#     [-1, -1, -1, -1]
# ]

mat = [
    [-1, -1, -1, -1, -1, -1, -1],
    [-1,  0,  0,  0, -1, -1, -1],
    [-1,  0,100,  0, -1, -1, -1],
    [-1,  0,  0,  0,  0,  0, -1],
    [-1, -1, -1,  0,100,  0, -1],
    [-1, -1, -1,  0,  0,  0, -1],
    [-1, -1, -1,  0,  0,  0, -1],
    [-1, -1, -1,  0,  0,  0, -1],
    [-1, -1, -1,  0,  0,  0, -1],
    [-1, -1, -1, -1, -1, -1, -1]
]



frontier_cell_list=[]

width=7
height=10

for r in range(0,height):
    for c in range(0,width):

        if r-1>=0:
            upper=mat[r-1][c]
        else:
            upper=False

        if r+1<height:
            lower=mat[r+1][c]
        else:
            lower=False

        if c-1>=0:
            left=mat[r][c-1]
        else:
            left=False

        if c+1<width:
            right=mat[r][c+1]
        else: 
            right=False
        


        
        if mat[r][c]==0:
                
                    if (upper==-1) or (lower==-1) or (right==-1) or (left==-1):
                        # cell={r,c}
                     frontier_cell_list.append((r,c))
                    #  selected_list.append(cell)
# print("\n".join(selected_list))
print(*frontier_cell_list,sep="\n")
# print(selected_list[0][0])
frontier_clustor=[]
temp_clustor=[]
# temp_clustor.append(frontier_cell_list[0])

# for index,cell in enumerate(frontier_cell_list):
#     r,c=cell
#     if index+1<len(frontier_cell_list):
#             nex_r,nex_c=frontier_cell_list[index+1]

#             # print(f"before entiering if :")
#             if (r==nex_r and nex_c-c==1):
#                     print(f"condidition matched {(r,c) , (nex_r,nex_c)}")
#                     temp_clustor.append((r,c))
#                     temp_clustor.append((nex_r,nex_c))
#             else:
                
#                 print(f"condition not match,{(r,c),(nex_r,nex_c)} entering else| temp_clustor{temp_clustor}")
#                 # print(temp_clustor)
#                 if temp_clustor:
#                     print(f"yes temp_clustor:{temp_clustor}")
#                     frontier_clustor.append(temp_clustor)
#                     print(f"frontier_closter after condition not match {frontier_clustor}")
#                     temp_clustor.clear()
                
#     else:
#          print(f"entere index else ,temp_cluster:{temp_clustor} frontier clustor:{frontier_clustor}")
#          frontier_clustor.append(temp_clustor)
#          print(f"last cell reached")

# print(f"final frontier cluster:{frontier_clustor}")

# for index, cell in enumerate(frontier_cell_list):
#     r, c = cell
#     if index + 1 < len(frontier_cell_list):
#         nex_r, nex_c = frontier_cell_list[index + 1]

#         if (r == nex_r and nex_c - c == 1):
#             if (r, c) not in temp_clustor:
#                 temp_clustor.append((r, c))
#             if (nex_r, nex_c) not in temp_clustor:
#                 temp_clustor.append((nex_r, nex_c))
#         else:
#             if temp_clustor:
#                 frontier_clustor.append(temp_clustor[:])
#                 temp_clustor = []
#             else:
#                 frontier_clustor.append([(r, c)])  # isolated cell

#     else:
#         if temp_clustor:
#             frontier_clustor.append(temp_clustor[:])
#         else:
#             frontier_clustor.append([(r, c)])  # last cell is isolated

# print(frontier_clustor)

    
            

    # elif c==nex_c:
    #     if nex_r-r==1:
    #         temp_clustor.append((r,c))
    #         temp_clustor.append((nex_r,nex_c))
            
            




            # break
    



    

        

# print(matrix
    #   )
    
# r=2
# c=1
# print(mat[r-1][c])
# print(mat[r][c])
# print(mat[r+1][c])

# print(mat[r][c-1])
# print(mat[r][c])
# print(mat[r][c+1])


