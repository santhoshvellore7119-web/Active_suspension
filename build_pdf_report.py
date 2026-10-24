"""
build_pdf_report.py
Renders the high-quality technical report template into PDF using Microsoft Edge headless.
"""

import os
import sys
import base64
import shutil
import subprocess
import pymupdf

def get_base64_image(path):
    if not os.path.exists(path):
        print(f"Warning: File not found: {path}")
        return ""
    ext = os.path.splitext(path)[1].lower().replace('.', '')
    if ext == 'jpg':
        ext = 'jpeg'
    with open(path, "rb") as f:
        encoded = base64.b64encode(f.read()).decode('utf-8')
    return f"data:image/{ext};base64,{encoded}"

def main():
    root_dir = os.path.abspath(os.path.dirname(__file__))
    docs_dir = os.path.join(root_dir, "docs")
    figs_dir = os.path.join(root_dir, "figures")
    
    template_path = os.path.join(docs_dir, "report_template.html")
    output_html   = os.path.join(docs_dir, "STSMC_and_Road_Profiles_Technical_Report.html")
    output_pdf    = os.path.join(docs_dir, "STSMC_and_Road_Profiles_Technical_Report.pdf")
    artifact_pdf  = r"C:\Users\Santhosh\.gemini\antigravity\brain\fc5050eb-d01c-48a6-8980-4f7c021b08aa\STSMC_and_Road_Profiles_Technical_Report.pdf"

    print("Reading HTML template...")
    with open(template_path, "r", encoding="utf-8") as f:
        html = f.read()

    print("Embedding high-resolution diagrams as base64...")
    replacements = {
        "__IMG_ROOT_MODEL__": get_base64_image(os.path.join(figs_dir, "Simulink_Root_Model.png")),
        "__IMG_ROAD_GEN__":   get_base64_image(os.path.join(figs_dir, "Simulink_Road_Surface_Generator.png")),
        "__IMG_STSMC_CTRL__": get_base64_image(os.path.join(figs_dir, "Simulink_STSMC_Controller.png")),
        "__IMG_CHIRP__":      get_base64_image(os.path.join(figs_dir, "Figure1_Chirp_Comparison.png")),
        "__IMG_SMOOTH__":     get_base64_image(os.path.join(figs_dir, "Figure2_Smooth_Comparison.png")),
        "__IMG_POTHOLE__":    get_base64_image(os.path.join(figs_dir, "Figure3_Pothole_Comparison.png")),
        "__IMG_RAMP__":       get_base64_image(os.path.join(figs_dir, "Figure4_Ramp_Comparison.png")),
        "__IMG_ROUGH__":      get_base64_image(os.path.join(figs_dir, "Figure5_Rough+Shock_Comparison.png")),
        "__IMG_RESONANCE__":  get_base64_image(os.path.join(figs_dir, "Figure6_Resonance_Comparison.png")),
    }

    for placeholder, data_uri in replacements.items():
        html = html.replace(placeholder, data_uri)

    with open(output_html, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"Saved standalone HTML report ({os.path.getsize(output_html) / 1024:.1f} KB)")

    edge_exe = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
    if not os.path.exists(edge_exe):
        edge_exe = r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"

    print(f"Printing PDF with Microsoft Edge: {edge_exe}")
    cmd = [
        edge_exe,
        "--headless=new",
        "--disable-gpu",
        "--run-all-compositor-stages-before-draw",
        "--virtual-time-budget=3000",
        "--no-pdf-header-footer",
        f"--print-to-pdf={output_pdf}",
        output_html
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    print(f"Edge process completed with exit code: {proc.returncode}")

    if os.path.exists(output_pdf):
        size_kb = os.path.getsize(output_pdf) / 1024
        print(f"SUCCESS: Created PDF at {output_pdf} ({size_kb:.1f} KB)")
        
        # Verify with pymupdf
        doc = pymupdf.open(output_pdf)
        num_pages = len(doc)
        print(f"PyMuPDF Verification: {num_pages} pages rendered successfully.")
        
        # Copy to brain artifact directory
        os.makedirs(os.path.dirname(artifact_pdf), exist_ok=True)
        shutil.copy2(output_pdf, artifact_pdf)
        print(f"Copied PDF to artifact directory: {artifact_pdf}")
    else:
        print("ERROR: Output PDF was not created!")
        print("STDOUT:", proc.stdout)
        print("STDERR:", proc.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
