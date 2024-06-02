# 打开原始的世界文件
with open('/home/maary/code/catkin_ws/src/mobile_manipulator/worlds/neighborhood.world', 'r') as f:
    lines = f.readlines()

# 缩放因子
scale_factor = 20

# 修改模型的大小和位置
for i, line in enumerate(lines):
    if '<model name="' in line:  # 定位到模型开始的行
        model_name = line.split('"')[1]  # 提取模型名称
        for j in range(i, len(lines)):
            if '</model>' in lines[j]:  # 定位到模型结束的行
                break
            if '<pose' in lines[j]:  # 定位到包含位置信息的行
                pose_str = lines[j].split('>')[1].split('<')[0]  # 提取位置信息字符串
                pose = [float(x) * scale_factor for x in pose_str.split()]  # 缩放位置
                lines[j] = '<pose> {} {} {} {} {} {} </pose>\n'.format(*pose)  # 生成新的位置信息行
            elif '<scale>' in lines[j]:  # 定位到包含尺寸信息的行
                scale_str = lines[j].split('>')[1].split('<')[0]  # 提取尺寸信息字符串
                scale = [float(x) * scale_factor for x in scale_str.split()]  # 缩放尺寸
                lines[j] = '<scale> {} {} {} </scale>\n'.format(*scale)  # 生成新的尺寸信息行

# 写入新的世界文件
with open('/home/maary/code/catkin_ws/src/mobile_manipulator/worlds/neighborhood_20x.world', 'w') as f:
    f.writelines(lines)
