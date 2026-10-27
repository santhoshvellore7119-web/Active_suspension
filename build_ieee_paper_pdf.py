"""
build_ieee_paper_pdf.py
Compiles IEEE_ACCESS_Paper_Active_Suspension_STSMC.html into a publication-quality PDF
using Microsoft Edge headless, embedding all 300 DPI figures as base64 data URIs.
"""

import os
import sys
import re
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
    
    html_src = os.path.join(docs_dir, "IEEE_ACCESS_Paper_Active_Suspension_STSMC.html")
    html_standalone = os.path.join(docs_dir, "IEEE_ACCESS_Paper_Active_Suspension_STSMC_standalone.html")
    output_pdf = os.path.join(docs_dir, "IEEE_ACCESS_Paper_Active_Suspension_STSMC.pdf")
    
    conversation_id = "9d9628c5-7574-49b5-b04e-877bd3c4aab2"
    artifact_dir = os.path.join(r"C:\Users\Santhosh\.gemini\antigravity\brain", conversation_id)
    artifact_pdf = os.path.join(artifact_dir, "IEEE_ACCESS_Paper_Active_Suspension_STSMC.pdf")

    print(f"Reading HTML source: {html_src}")
    with open(html_src, "r", encoding="utf-8") as f:
        html = f.read()

    # Embed all local images as base64 data URIs
    print("Embedding all figures as base64 data URIs...")
    def replace_img_src(match):
        rel_path = match.group(1)
        # Normalize relative path
        fname = os.path.basename(rel_path)
        full_p = os.path.join(figs_dir, fname)
        if os.path.exists(full_p):
            b64_uri = get_base64_image(full_p)
            return f"src='{b64_uri}'"
        return match.group(0)

    html = re.sub(r"src=['\"](?:\.\./figures/|figures/)?([^'\"]+\.png)['\"]", replace_img_src, html)

    # Enhance print CSS for clean pagination and margins
    custom_print_css = """
    <style>
    @page {
        size: letter;
        margin: 18mm 16mm 20mm 16mm;
        @bottom-right {
            content: "Page " counter(page);
            font-size: 8.5pt;
            color: #64748b;
            font-family: Arial, sans-serif;
        }
        @top-right {
            content: "IEEE Access 2026 - Active Suspension STSMC Benchmark";
            font-size: 8pt;
            color: #94a3b8;
            font-family: Arial, sans-serif;
        }
    }
    body {
        font-family: 'Times New Roman', Times, serif;
        font-size: 10pt;
        line-height: 1.5;
        color: #111827;
    }
    .figure-box {
        page-break-inside: avoid;
        margin: 16px 0;
        text-align: center;
    }
    .figure-box img {
        max-width: 95%;
        height: auto;
    }
    .table-responsive, table {
        page-break-inside: avoid;
    }
    h1, h2, h3, h4 {
        page-break-after: avoid;
    }
    h2.section-title {
        border-bottom: 2px solid #0b2545;
        color: #0b2545;
        padding-bottom: 4px;
        margin-top: 24px;
        font-size: 13pt;
        letter-spacing: 0.5px;
    }
    .paper-title {
        color: #0b2545;
        font-size: 18pt;
        line-height: 1.3;
        margin-bottom: 12px;
    }
    </style>
    """
    html = html.replace("</head>", f"{custom_print_css}\n</head>")

    with open(html_standalone, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"Saved standalone HTML ({os.path.getsize(html_standalone) / 1024:.1f} KB)")

    edge_exe = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
    if not os.path.exists(edge_exe):
        edge_exe = r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"

    print(f"Compiling PDF with Edge: {edge_exe}")
    cmd = [
        edge_exe,
        "--headless=new",
        "--disable-gpu",
        "--run-all-compositor-stages-before-draw",
        "--virtual-time-budget=6000",
        "--no-pdf-header-footer",
        f"--print-to-pdf={output_pdf}",
        html_standalone
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    print(f"Edge process exited with code: {proc.returncode}")

    if os.path.exists(output_pdf):
        size_kb = os.path.getsize(output_pdf) / 1024
        print(f"SUCCESS: Created PDF at {output_pdf} ({size_kb:.1f} KB)")

        # Verify with pymupdf
        doc = pymupdf.open(output_pdf)
        num_pages = len(doc)
        print(f"PyMuPDF Verification: {num_pages} pages rendered successfully.")
        
        # Extract page 1 thumbnail to verify formatting
        page1 = doc[0]
        pix = page1.get_pixmap(dpi=150)
        thumb_path = os.path.join(artifact_dir, "ieee_paper_page1_thumb.png")
        pix.save(thumb_path)
        print(f"Saved Page 1 thumbnail: {thumb_path}")

        # Copy to conversation artifact directory
        os.makedirs(artifact_dir, exist_ok=True)
        shutil.copy2(output_pdf, artifact_pdf)
        print(f"Copied PDF to artifact directory: {artifact_pdf}")
    else:
        print("ERROR: PDF was not generated!")
        print("STDOUT:", proc.stdout)
        print("STDERR:", proc.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
