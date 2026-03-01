import matplotlib.pyplot as plt  
from mpl_toolkits.mplot3d import Axes3D  

# 直接定义输入的坐标数据为数组形式  
data_array = [
    11.7762, 6.99451,
    11.7355, 2.82593,
    10.4638, 1.96768,
    11.232, 0.166951,
    10.515, -1.55228,
    4.82919, 2.74793,
    7.4812, 6.88453, 
    9.22459 , 5.87239
]  

def parse_coordinates3(data_array):  
    # 按每三个数分组为坐标  
    return [(data_array[i], data_array[i + 1], data_array[i + 2]) for i in range(0, len(data_array), 3)]  

def parse_coordinates2(data_array):  
    # 按每2个数分组为坐标  
    return [(data_array[i], data_array[i + 1], 0) for i in range(0, len(data_array), 2)]  

def plot_points(points):  
    # 提取x, y, z坐标  
    xs, ys, zs = zip(*points)  
    
    # 创建三维图形  
    fig = plt.figure()  
    ax = fig.add_subplot(111, projection='3d')  
    
    # 绘制散点  
    ax.scatter(xs, ys, zs, c='b', marker='o')  
    
    ax.set_xlabel('X Label')  
    ax.set_ylabel('Y Label')  
    ax.set_zlabel('Z Label')  
    ax.set_title('3D Coordinate Visualization')  
    
    plt.show()  

def main():  
    points = parse_coordinates2(data_array)  
    plot_points(points)  

if __name__ == "__main__":  
    main()  