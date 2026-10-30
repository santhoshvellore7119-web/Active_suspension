"""
generate_final_publication_paper.py
Author: Santhosh G, Thilang T.V, Ananthakrishnan R
SELECT, Vellore Institute of Technology (VIT), Vellore, India

Generates the complete 11-page publication-grade IEEE Access research paper:
'Second-Order Super-Twisting Sliding Mode Control With Adaptive Road Severity
Gain Scheduling for Vehicle Active Suspension Systems'
"""

import os
import sys
import base64
import subprocess
import pymupdf
import shutil

def get_base64_image(path):
    if not os.path.exists(path):
        print(f"Warning: image path not found: {path}")
        return ""
    ext = os.path.splitext(path)[1].lower().replace('.', '')
    if ext == 'jpg': ext = 'jpeg'
    with open(path, "rb") as f:
        encoded = base64.b64encode(f.read()).decode('utf-8')
    return f"data:image/{ext};base64,{encoded}"

def make_avatar_svg(initials, bg_color):
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="120" height="150" viewBox="0 0 120 150">
        <rect width="120" height="150" fill="{bg_color}"/>
        <circle cx="60" cy="55" r="28" fill="#ffffff" opacity="0.3"/>
        <path d="M20 135 C20 95, 100 95, 100 135 Z" fill="#ffffff" opacity="0.3"/>
        <text x="60" y="65" font-family="'Segoe UI', Arial, sans-serif" font-size="22" font-weight="bold" fill="#ffffff" text-anchor="middle">{initials}</text>
    </svg>'''
    return f"data:image/svg+xml;base64,{base64.b64encode(svg.encode('utf-8')).decode('utf-8')}"

def main():
    root_dir = os.path.abspath(r"c:\Users\Santhosh\Documents\antigravity\zealous-borg")
    docs_dir = os.path.join(root_dir, "docs")
    res_dir  = os.path.join(root_dir, "results")
    curve_dir = os.path.join(res_dir, "tuning_curves")
    art_dir  = r"C:\Users\Santhosh\.gemini\antigravity\brain\9d9628c5-7574-49b5-b04e-877bd3c4aab2"

    os.makedirs(docs_dir, exist_ok=True)
    os.makedirs(art_dir, exist_ok=True)

    # 1. Base64 Images
    simulink_img_path = os.path.join(art_dir, ".user_uploaded", "media_1793271139864_1a6604b4.png")
    img_simulink = get_base64_image(simulink_img_path)
    img_fig1 = get_base64_image(os.path.join(res_dir, "Figure1_Simulation_Response.png"))
    img_fig2 = get_base64_image(os.path.join(res_dir, "Figure2_Simulation_Response.png"))
    img_fig3 = get_base64_image(os.path.join(res_dir, "Figure3_Simulation_Response.png"))
    img_fig4 = get_base64_image(os.path.join(res_dir, "Figure4_Simulation_Response.png"))
    img_fig5 = get_base64_image(os.path.join(res_dir, "Figure5_Simulation_Response.png"))
    img_fig6 = get_base64_image(os.path.join(res_dir, "Figure6_Simulation_Response.png"))
    img_dashboard = get_base64_image(os.path.join(res_dir, "Figure_Candidate5_All_Roads.png"))
    img_rsi = get_base64_image(os.path.join(curve_dir, "Continuous_RSI_Scheduler_Curves.png"))

    img_bio1 = make_avatar_svg("TV", "#1e3a8a")
    img_bio2 = make_avatar_svg("SG", "#065f46")
    img_bio3 = make_avatar_svg("AR", "#7c2d12")

    # Template replacement tokens
    replacements = {
        "__IMG_SIMULINK__": img_simulink,
        "__IMG_RSI__": img_rsi,
        "__IMG_FIG1__": img_fig1,
        "__IMG_FIG2__": img_fig2,
        "__IMG_FIG3__": img_fig3,
        "__IMG_FIG4__": img_fig4,
        "__IMG_FIG5__": img_fig5,
        "__IMG_FIG6__": img_fig6,
        "__IMG_DASHBOARD__": img_dashboard,
        "__IMG_BIO1__": img_bio1,
        "__IMG_BIO2__": img_bio2,
        "__IMG_BIO3__": img_bio3
    }

    # Load HTML template from separate file to completely avoid python escaping issues
    template_path = os.path.join(root_dir, "paper_template.html")
    if not os.path.exists(template_path):
        print(f"Error: {template_path} does not exist!")
        return

    with open(template_path, "r", encoding="utf-8") as f:
        html_content = f.read()

    for k, v in replacements.items():
        html_content = html_content.replace(k, v)

    html_out = os.path.join(docs_dir, "IEEE_Access_Active_Suspension_STSMC_Paper.html")
    pdf_out  = os.path.join(docs_dir, "IEEE_Access_Active_Suspension_STSMC_Paper.pdf")
    art_html = os.path.join(art_dir, "IEEE_Access_Active_Suspension_STSMC_Paper.html")
    art_pdf  = os.path.join(art_dir, "IEEE_Access_Active_Suspension_STSMC_Paper.pdf")

    with open(html_out, "w", encoding="utf-8") as f:
        f.write(html_content)
    shutil.copyfile(html_out, art_html)
    print(f"Wrote HTML paper to {html_out} ({os.path.getsize(html_out)/1024:.1f} KB)")

    edge_exe = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
    if not os.path.exists(edge_exe):
        edge_exe = r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"

    print(f"Rendering publication PDF via Edge Headless: {edge_exe}")
    cmd = [
        edge_exe,
        "--headless=new",
        "--disable-gpu",
        "--run-all-compositor-stages-before-draw",
        "--virtual-time-budget=10000",
        "--no-pdf-header-footer",
        f"--print-to-pdf={pdf_out}",
        f"file:///{html_out.replace(os.sep, '/')}"
    ]

    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        print("Edge rendering error:", res.stderr)
        sys.exit(1)

    shutil.copyfile(pdf_out, art_pdf)
    print(f"Successfully generated PDF: {pdf_out} ({os.path.getsize(pdf_out)/1024:.1f} KB)")

    # PyMuPDF Inspection & Thumbnails
    doc = pymupdf.open(pdf_out)
    num_pages = len(doc)
    print(f"Total Pages in Generated IEEE Access Paper: {num_pages}")
    for i in range(num_pages):
        page = doc[i]
        pix = page.get_pixmap(dpi=150)
        pix.save(os.path.join(art_dir, f"paper_page_{i+1}_thumb.png"))
    print(f"Exported {num_pages} high-res page thumbnails to artifact directory.")
    doc.close()

if __name__ == "__main__":
    main()
