"""
POLISH_SIMULINK_CANVAS.PY
Polishes Master_Active_Suspension_Clean.slx directly in the XML structure:
  - Updates all titles and annotations cleanly (1. Optimal LQR, 2. Fixed STSMC, 3. Adaptive STSMC)
  - Eliminates overlapping text annotations (zr_dot, Fc)
  - Adds master title header
  - Properly spaces RMS subsystems and displays
  - Fixes the Integrator lower saturation limit to 0 to prevent sqrt(-eps) warnings
"""

import os
import zipfile
import shutil
import xml.etree.ElementTree as ET

slx_file = 'Master_Active_Suspension_Clean.slx'
if not os.path.exists(slx_file):
    raise FileNotFoundError(f"Missing {slx_file}")

temp_dir = 'temp_slx_polish'
if os.path.exists(temp_dir):
    shutil.rmtree(temp_dir)
os.makedirs(temp_dir, exist_ok=True)

with zipfile.ZipFile(slx_file, 'r') as z:
    z.extractall(temp_dir)

# 1. Edit simulink/systems/system_root.xml
root_xml_path = os.path.join(temp_dir, 'simulink', 'systems', 'system_root.xml')
tree = ET.parse(root_xml_path)
root = tree.getroot()

# Update Annotations
annotations_to_remove = []
for a in root.findall('.//Annotation'):
    sid = a.get('SID')
    
    # Remove overlapping floating labels zr_dot and Fc
    if sid in ['178', '179', '180', '182', '183', '186']:
        annotations_to_remove.append(a)
        continue
        
    for p in a.findall('P'):
        p_name = p.get('Name')
        if sid == '177': # 1. ACO -> 1. OPTIMAL LQR CONTROLLER
            if p_name == 'Name':
                p.text = '<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd"><html><head><meta name="qrichtext" content="1" /><style type="text/css">p, li { white-space: pre-wrap; }</style></head><body align="left" style=" font-family:\'Arial\'; font-size:12px; font-weight:600; font-style:normal;"><p align="left" style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:13px; font-weight:bold; color:#003366;">1. OPTIMAL LQR CONTROLLER (Active Baseline)</span></p></body></html>'
            elif p_name == 'Position':
                p.text = '[410, 16, 730, 36]'
        elif sid == '187': # 2. HS -> 2. FIXED SUPER-TWISTING SMC
            if p_name == 'Name':
                p.text = '<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd"><html><head><meta name="qrichtext" content="1" /><style type="text/css">p, li { white-space: pre-wrap; }</style></head><body align="left" style=" font-family:\'Arial\'; font-size:12px; font-weight:600; font-style:normal;"><p align="left" style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:13px; font-weight:bold; color:#994400;">2. FIXED SUPER-TWISTING SMC (Constant Robust Gains)</span></p></body></html>'
            elif p_name == 'Position':
                p.text = '[390, 250, 750, 270]'
        elif sid == '185': # 3. Classical -> 3. ADAPTIVE SUPER-TWISTING SMC
            if p_name == 'Name':
                p.text = '<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd"><html><head><meta name="qrichtext" content="1" /><style type="text/css">p, li { white-space: pre-wrap; }</style></head><body align="left" style=" font-family:\'Arial\'; font-size:12px; font-weight:600; font-style:normal;"><p align="left" style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:13px; font-weight:bold; color:#006622;">3. ADAPTIVE SUPER-TWISTING SMC (RSI Dynamic Gain-Scheduled)</span></p></body></html>'
            elif p_name == 'Position':
                p.text = '[350, 500, 770, 520]'
        elif sid == '188': # ACCELERATION
            if p_name == 'Name':
                p.text = '<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd"><html><head><meta name="qrichtext" content="1" /><style type="text/css">p, li { white-space: pre-wrap; }</style></head><body style=" font-family:\'Arial\'; font-size:10px; font-weight:400; font-style:normal;"><p style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:12px; font-weight:600; color:#b30000;">BODY ACCELERATION [Ride Comfort]</span></p></body></html>'
            elif p_name == 'Position':
                p.text = '[920, 320, 1180, 340]'
        elif sid == '181': # SUSPENSION DEFLECTION
            if p_name == 'Name':
                p.text = '<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd"><html><head><meta name="qrichtext" content="1" /><style type="text/css">p, li { white-space: pre-wrap; }</style></head><body style=" font-family:\'Arial\'; font-size:10px; font-weight:400; font-style:normal;"><p style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:12px; font-weight:600; color:#005580;">SUSPENSION DEFLECTION [+/-38 mm Limit]</span></p></body></html>'
            elif p_name == 'Position':
                p.text = '[920, 500, 1190, 520]'
        elif sid == '184': # CONTROL FORCE
            if p_name == 'Name':
                p.text = '<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd"><html><head><meta name="qrichtext" content="1" /><style type="text/css">p, li { white-space: pre-wrap; }</style></head><body style=" font-family:\'Arial\'; font-size:10px; font-weight:400; font-style:normal;"><p style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:12px; font-weight:600; color:#4d0099;">ACTUATOR CONTROL FORCE [+/-39.2 N Saturation]</span></p></body></html>'
            elif p_name == 'Position':
                p.text = '[920, 650, 1220, 670]'
        elif sid == '201': # RMS
            if p_name == 'Name':
                p.text = 'REAL-TIME IN-SIMULINK RMS PERFORMANCE READOUTS (mm)'
            elif p_name == 'Position':
                p.text = '[940, 420, 1340, 436]'

# Remove overlapping floating annotations
for a in annotations_to_remove:
    root.remove(a)

# Add Master Header Title Annotation
hdr_ann = ET.SubElement(root, 'Annotation', {'SID': '9999'})
hdr_name = ET.SubElement(hdr_ann, 'P', {'Name': 'Name'})
hdr_name.text = '<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd"><html><head><meta name="qrichtext" content="1" /><style type="text/css">p, li { white-space: pre-wrap; }</style></head><body align="left" style=" font-family:\'Arial\'; font-size:14px; font-weight:600; font-style:normal;"><p align="left" style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:16px; font-weight:bold; color:#002244;">QUANSER 2-DOF QUARTER-CAR ACTIVE SUSPENSION BENCHMARK (3-CONTROLLER STUDY)</span></p></body></html>'
hdr_pos = ET.SubElement(hdr_ann, 'P', {'Name': 'Position'})
hdr_pos.text = '[240, -10, 880, 15]'
hdr_int = ET.SubElement(hdr_ann, 'P', {'Name': 'Interpreter'})
hdr_int.text = 'rich'

# Adjust Data_Logging position so it is spacious and clean
for b in root.findall('.//Block'):
    name = b.get('Name')
    if name == 'Data_Logging':
        for p in b.findall('P'):
            if p.get('Name') == 'Position':
                p.text = '[380, 840, 580, 930]'

tree.write(root_xml_path, encoding='utf-8', xml_declaration=True)
print("Updated system_root.xml with polished annotations and layout.")

# 2. Fix Integrators in RMS calculation blocks (system_198, system_202, system_209)
for sys_file in ['system_198.xml', 'system_202.xml', 'system_209.xml']:
    p_path = os.path.join(temp_dir, 'simulink', 'systems', sys_file)
    if not os.path.exists(p_path):
        continue
    tree_s = ET.parse(p_path)
    root_s = tree_s.getroot()
    for b in root_s.findall('.//Block'):
        if b.get('BlockType') == 'Integrator':
            # Add or update LimitOutput = 'on', LowerSaturationLimit = '0'
            lim_p = None
            low_p = None
            for p in b.findall('P'):
                if p.get('Name') == 'LimitOutput': lim_p = p
                elif p.get('Name') == 'LowerSaturationLimit': low_p = p
            if lim_p is not None: lim_p.text = 'on'
            else:
                lim_p = ET.SubElement(b, 'P', {'Name': 'LimitOutput'})
                lim_p.text = 'on'
            if low_p is not None: low_p.text = '0'
            else:
                low_p = ET.SubElement(b, 'P', {'Name': 'LowerSaturationLimit'})
                low_p.text = '0'
    tree_s.write(p_path, encoding='utf-8', xml_declaration=True)
    print(f"Fixed Integrator bounds in {sys_file}")

# 3. Rebuild ZIP into Master_Active_Suspension_Clean.slx
backup_slx = 'Master_Active_Suspension_Clean_backup.slx'
shutil.copyfile(slx_file, backup_slx)

with zipfile.ZipFile(slx_file, 'w', compression=zipfile.ZIP_DEFLATED) as z_out:
    for foldername, subfolders, filenames in os.walk(temp_dir):
        for filename in filenames:
            filepath = os.path.join(foldername, filename)
            arcname = os.path.relpath(filepath, temp_dir)
            z_out.write(filepath, arcname)

# Also update Master_Active_Suspension_Simulation.slx
shutil.copyfile(slx_file, 'Master_Active_Suspension_Simulation.slx')
print("Polished Master_Active_Suspension_Clean.slx & Master_Active_Suspension_Simulation.slx created successfully!")

# Clean up temp directory
shutil.rmtree(temp_dir)
