import re
import os

def markdown_to_html(md_path, html_path):
    with open(md_path, 'r', encoding='utf-8') as f:
        md_text = f.read()

    # Pre-extract math expressions to avoid markdown collision
    display_math = []
    def save_display_math(match):
        display_math.append(match.group(1))
        return f"@@DISPLAYMATH_{len(display_math)-1}@@"
    
    md_text = re.sub(r'\$\$(.*?)\$\$', save_display_math, md_text, flags=re.DOTALL)

    inline_math = []
    def save_inline_math(match):
        inline_math.append(match.group(1))
        return f"@@INLINEMATH_{len(inline_math)-1}@@"
    
    md_text = re.sub(r'\$(.*?)\$', save_inline_math, md_text)

    # Pre-extract mermaid blocks
    mermaid_blocks = []
    def save_mermaid(match):
        mermaid_blocks.append(match.group(1).strip())
        return f"@@MERMAID_{len(mermaid_blocks)-1}@@"
    
    md_text = re.sub(r'```mermaid(.*?)```', save_mermaid, md_text, flags=re.DOTALL)

    # Process markdown line by line
    lines = md_text.split('\n')
    out_lines = []
    in_table = False
    table_lines = []

    def flush_table():
        nonlocal in_table, table_lines
        if not table_lines:
            return
        html_t = ["<div class='table-responsive'><table>"]
        header_done = False
        for i, row in enumerate(table_lines):
            cells = [c.strip() for c in row.strip('|').split('|')]
            if all(re.match(r'^:?-+:?$', c) for c in cells if c):
                header_done = True
                continue
            if not header_done:
                html_t.append("<thead><tr>" + "".join(f"<th>{c}</th>" for c in cells) + "</tr></thead><tbody>")
            else:
                html_t.append("<tr>" + "".join(f"<td>{c}</td>" for c in cells) + "</tr>")
        html_t.append("</tbody></table></div>")
        out_lines.append("\n".join(html_t))
        table_lines = []
        in_table = False

    for line in lines:
        sline = line.strip()
        if sline.startswith('|') and sline.endswith('|'):
            in_table = True
            table_lines.append(sline)
            continue
        elif in_table:
            flush_table()

        # Headings
        if sline.startswith('# '):
            out_lines.append(f"<h1 class='paper-title'>{sline[2:]}</h1>")
        elif sline.startswith('## '):
            out_lines.append(f"<h2 class='section-title'>{sline[3:]}</h2>")
        elif sline.startswith('### '):
            out_lines.append(f"<h3 class='subsection-title'>{sline[4:]}</h3>")
        elif sline.startswith('#### '):
            out_lines.append(f"<h4 class='subsubsection-title'>{sline[5:]}</h4>")
        elif sline == '---':
            out_lines.append("<hr class='paper-hr'>")
        elif sline.startswith('- ') or sline.startswith('* '):
            out_lines.append(f"<li>{sline[2:]}</li>")
        elif re.match(r'^\d+\.\s', sline):
            item_text = re.sub(r'^\d+\.\s', '', sline)
            out_lines.append(f"<li class='ordered-item'>{item_text}</li>")
        elif sline.startswith('> '):
            out_lines.append(f"<blockquote>{sline[2:]}</blockquote>")
        elif re.match(r'^!\[(.*?)\]\((.*?)\)$', sline):
            m = re.match(r'^!\[(.*?)\]\((.*?)\)$', sline)
            caption, img_url = m.group(1), m.group(2)
            out_lines.append(f"<div class='figure-box' style='text-align:center; margin:18px 0;'><img src='{img_url}' alt='{caption}' style='max-width:100%; height:auto; border:1px solid #e2e8f0; border-radius:4px;'><p class='figure-caption' style='font-size:9.5pt; color:#334155; margin-top:6px; font-weight:600;'>{caption}</p></div>")
        elif sline == '':
            out_lines.append("<div class='spacer'></div>")
        else:
            out_lines.append(f"<p>{line}</p>")

    if in_table:
        flush_table()

    html_body = "\n".join(out_lines)

    # Bold and italics
    html_body = re.sub(r'\*\*(.*?)\*\*', r'<strong>\1</strong>', html_body)
    html_body = re.sub(r'\*(.*?)\*', r'<em>\1</em>', html_body)

    # Restore mermaid
    for i, m_code in enumerate(mermaid_blocks):
        html_body = html_body.replace(f"@@MERMAID_{i}@@", f"<div class='mermaid'>{m_code}</div>")

    # Restore math
    for i, m_math in enumerate(display_math):
        html_body = html_body.replace(f"@@DISPLAYMATH_{i}@@", f"$${m_math}$$")

    for i, m_math in enumerate(inline_math):
        html_body = html_body.replace(f"@@INLINEMATH_{i}@@", f"${m_math}$")

    full_html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Second-Order Super-Twisting Sliding Mode Control Versus Metaheuristic-Tuned LQR for Active Suspension Systems</title>

<!-- KaTeX for math rendering -->
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.css">
<script defer src="https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.js"></script>
<script defer src="https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/contrib/auto-render.min.js"
    onload="renderMathInElement(document.body, {{delimiters: [{{left: '$$', right: '$$', display: true}}, {{left: '$', right: '$', display: false}}]}});">
</script>

<!-- Mermaid.js for architecture diagrams -->
<script src="https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js"></script>
<script>mermaid.initialize({{startOnLoad:true, theme:'neutral'}});</script>

<style>
@page {{
    size: letter;
    margin: 20mm 15mm;
}}
body {{
    font-family: 'Times New Roman', Times, serif, Cambria;
    color: #111;
    background-color: #fff;
    line-height: 1.5;
    font-size: 10.5pt;
    max-width: 950px;
    margin: 0 auto;
    padding: 30px 20px;
}}
.paper-title {{
    font-size: 20pt;
    font-weight: bold;
    text-align: center;
    color: #0b2545;
    margin-bottom: 12px;
    line-height: 1.25;
}}
.section-title {{
    font-size: 13pt;
    font-weight: bold;
    text-transform: uppercase;
    color: #0b2545;
    border-bottom: 1.5px solid #0b2545;
    padding-bottom: 3px;
    margin-top: 24px;
    margin-bottom: 10px;
}}
.subsection-title {{
    font-size: 11pt;
    font-weight: bold;
    color: #134074;
    margin-top: 16px;
    margin-bottom: 6px;
}}
.subsubsection-title {{
    font-size: 10.5pt;
    font-weight: bold;
    font-style: italic;
    color: #1d3557;
    margin-top: 12px;
}}
p {{
    text-align: justify;
    margin-top: 6px;
    margin-bottom: 8px;
    text-indent: 1.5em;
}}
p strong, p em {{
    text-indent: 0;
}}
.table-responsive {{
    overflow-x: auto;
    margin: 16px 0;
}}
table {{
    width: 100%;
    border-collapse: collapse;
    font-size: 9pt;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    margin-top: 8px;
    margin-bottom: 8px;
}}
th, td {{
    border: 1px solid #cbd5e1;
    padding: 6px 9px;
    text-align: left;
}}
th {{
    background-color: #f1f5f9;
    color: #0f172a;
    font-weight: 600;
}}
tr:nth-child(even) {{
    background-color: #f8fafc;
}}
.paper-hr {{
    border: none;
    border-top: 1px solid #94a3b8;
    margin: 18px 0;
}}
blockquote {{
    border-left: 3px solid #2563eb;
    margin: 10px 0;
    padding: 6px 14px;
    background-color: #f0f7ff;
    font-style: italic;
}}
.mermaid {{
    text-align: center;
    margin: 16px 0;
    background: #f8fafc;
    padding: 10px;
    border-radius: 6px;
    border: 1px solid #e2e8f0;
}}
li {{
    margin-bottom: 4px;
    margin-left: 20px;
}}
.spacer {{
    height: 4px;
}}
@media print {{
    body {{
        max-width: 100%;
        padding: 0;
    }}
    .mermaid {{
        page-break-inside: avoid;
    }}
    table {{
        page-break-inside: avoid;
    }}
}}
</style>
</head>
<body>
{html_body}
</body>
</html>"""

    with open(html_path, 'w', encoding='utf-8') as f:
        f.write(full_html)
    print(f"Generated {html_path} successfully ({len(full_html)} bytes)")

if __name__ == '__main__':
    md_file = os.path.join(os.path.dirname(__file__), 'docs', 'IEEE_ACCESS_Paper_Active_Suspension_STSMC.md')
    html_file = os.path.join(os.path.dirname(__file__), 'docs', 'IEEE_ACCESS_Paper_Active_Suspension_STSMC.html')
    markdown_to_html(md_file, html_file)
