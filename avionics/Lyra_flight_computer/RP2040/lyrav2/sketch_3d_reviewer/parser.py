f = open("input.csv","r")

import csv
content = list(csv.reader(f,delimiter=","));

output = []
outstring = ''
l = 0
for i in content:
    output.append([0.0,0.0,0.0,0.0])
    output[l][0] = i[1]
    output[l][1] = i[17]
    output[l][2] = i[18]
    output[l][3] = i[19]
    outstring = outstring + "{" + ','.join(output[l]) + "},\n"
    l = l+1
    
f.close()

fd = open("output.csv","w+")
fd.write(outstring)
# print(content)
# print(output)
print(outstring)

