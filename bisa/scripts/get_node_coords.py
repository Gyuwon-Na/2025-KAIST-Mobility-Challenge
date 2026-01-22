#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
노드 좌표 확인 스크립트 - 회전교차로 설정을 위해 사용
실행: python3 get_node_coords.py
"""

import os
import sys

# 경로 설정
sys.path.insert(0, os.path.expanduser('~/Mobility_Challenge_Simulator/src/bisa/scripts'))
from mgeo_class_defs import MGeoPlannerMap

def main():
    # HD Map 로드
    load_path = os.path.expanduser('~/Mobility_Challenge_Simulator/src/bisa/hdmap_data')
    
    if not os.path.exists(load_path):
        load_path = os.path.expanduser('~/Mobility_Challenge_Simulator/install/bisa/share/bisa/hdmap_data')
    
    print(f"Loading from: {load_path}")
    mgeo = MGeoPlannerMap.create_instance_from_json(load_path)
    
    # 회전교차로 진입 노드들
    entry_nodes = [36, 37, 46, 47]
    
    # 회전교차로 내부 노드들 (추정)
    roundabout_nodes = [38, 39, 40, 41, 42, 43, 48, 49]
    
    print("\n" + "="*60)
    print("회전교차로 진입 노드 좌표")
    print("="*60)
    
    for node_num in entry_nodes:
        key = f"NODE_{node_num}"
        if key in mgeo.node_set.nodes:
            node = mgeo.node_set.nodes[key]
            x, y, z = node.point
            print(f"NODE_{node_num}: ({x:.4f}, {y:.4f}, {z:.4f})")
        else:
            print(f"NODE_{node_num}: NOT FOUND")
    
    print("\n" + "="*60)
    print("회전교차로 내부 노드 좌표 (추정)")
    print("="*60)
    
    for node_num in roundabout_nodes:
        key = f"NODE_{node_num}"
        if key in mgeo.node_set.nodes:
            node = mgeo.node_set.nodes[key]
            x, y, z = node.point
            print(f"NODE_{node_num}: ({x:.4f}, {y:.4f}, {z:.4f})")
    
    # 회전교차로 중심 계산
    print("\n" + "="*60)
    print("회전교차로 중심 (내부 노드 평균)")
    print("="*60)
    
    xs, ys = [], []
    for node_num in roundabout_nodes:
        key = f"NODE_{node_num}"
        if key in mgeo.node_set.nodes:
            node = mgeo.node_set.nodes[key]
            xs.append(node.point[0])
            ys.append(node.point[1])
    
    if xs:
        center_x = sum(xs) / len(xs)
        center_y = sum(ys) / len(ys)
        print(f"Center: ({center_x:.4f}, {center_y:.4f})")
        
        # 반경 계산 (중심에서 가장 먼 노드까지 거리)
        import math
        max_dist = 0
        for x, y in zip(xs, ys):
            dist = math.sqrt((x - center_x)**2 + (y - center_y)**2)
            max_dist = max(max_dist, dist)
        print(f"Estimated Radius: {max_dist:.4f}m")
    
    print("\n" + "="*60)
    print("전체 노드 목록")
    print("="*60)
    
    for key, node in sorted(mgeo.node_set.nodes.items()):
        x, y, z = node.point
        print(f"{key}: ({x:.4f}, {y:.4f})")

if __name__ == '__main__':
    main()
