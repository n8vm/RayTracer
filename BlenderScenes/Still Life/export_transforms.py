import bpy

def grados(radianes):
    x= (180*radianes)/3.141516
    return x


#open path_file where I will save
full_filel = bpy.data.filepath.split("\\")
#print(full_filel)
#print(len(full_filel))
contador = 1 
path_file = full_filel[0]

while contador != len(full_filel)-1:
    path_file = str(path_file) + "\\" + str(full_filel[contador])
    contador +=1

#path_file actual
print(path_file)

#file name

file_name = "data.txt"

#path blend file
file1 = open((path_file + '\\' + file_name), 'w', encoding = "utf-8")

objects = bpy.context.scene.objects

for ob in objects:
    if ob.type =="MESH":
        name = ob.name
        loc = tuple(ob.location)
        sc = tuple(ob.scale)
        rot = [grados(ob.rotation_euler[0]), grados(ob.rotation_euler[1]), grados(ob.rotation_euler[2])]

        print(name, " ", loc, " ", sc, " ", rot)

        dato = (ob.name," Location: ", loc, " Scale: ", sc, " Rotation: ", rot )
        #write data in file
        file1.write(str(dato) + "\n")


#close the file
file1.close()