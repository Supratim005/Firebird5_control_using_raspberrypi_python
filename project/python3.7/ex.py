data = []
device_id = 2
device_type = 1
function_type = 9
param_count = 2
param_1 = 0
param_2 = 255
data.append(chr(device_id))
data.append(chr(device_type))
data.append(chr(function_type))
data.append(chr(param_count))
data.append((param_1)) 
data.append((param_2))
data.append("\n")
data.append("\r")

for i in range(0,len(data)):
    if i==4 :
        if data[i]>15:
           print(data[i])
           print(str(data[i]))
           print((str(hex(data[i]))).encode())
        else:
            print(data[i])
            print(str(data[i]))
            print((str(chr(data[i]))).encode())
    elif i==5:
        if data[i]>15:
            x=bytes([(data[i])]);
            #y= [1:]
            
            print(bytes([data[i]]))
            print(x)
            print(str(x))
            #print(f.write(x))
        
        else:
            print(data[i])
            print(str(data[i]))
            print((str(chr(data[i]))).encode())
    else:
        print(data[i])
        print(str(data[i]))
        print((str((data[i]))).encode())


    print(1)
    
print("packet sent is ", str(data))