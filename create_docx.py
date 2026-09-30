import os
import docx
from docx import Document
from docx.shared import Pt, RGBColor, Inches, Cm
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT, WD_ALIGN_VERTICAL
from docx.oxml import OxmlElement, parse_xml
from docx.oxml.ns import qn, nsdecls

def set_cell_background(cell, fill_hex):
    tcPr = cell._tc.get_or_add_tcPr()
    shd = parse_xml(f'<w:shd {nsdecls("w")} w:fill="{fill_hex}"/>')
    tcPr.append(shd)

def set_cell_margins(cell, top=100, bottom=100, left=150, right=150):
    tcPr = cell._tc.get_or_add_tcPr()
    tcMar = parse_xml(f'''
        <w:tcMar {nsdecls("w")}>
            <w:top w:w="{top}" w:type="dxa"/>
            <w:bottom w:w="{bottom}" w:type="dxa"/>
            <w:left w:w="{left}" w:type="dxa"/>
            <w:right w:w="{right}" w:type="dxa"/>
        </w:tcMar>
    ''')
    tcPr.append(tcMar)

def set_cell_borders(cell, top=None, bottom=None, left=None, right=None):
    tcPr = cell._tc.get_or_add_tcPr()
    borders = ['<w:tcBorders ' + nsdecls('w') + '>']
    for side, border in [('top', top), ('bottom', bottom), ('left', left), ('right', right)]:
        if border:
            borders.append(f'<w:{side} w:val="{border.get("val", "single")}" w:sz="{border.get("sz", "4")}" w:space="0" w:color="{border.get("color", "CBD5E1")}"/>')
        else:
            borders.append(f'<w:{side} w:val="none"/>')
    borders.append('</w:tcBorders>')
    tcBorders = parse_xml(''.join(borders))
    tcPr.append(tcBorders)

def set_paragraph_rtl(p):
    pPr = p._p.get_or_add_pPr()
    bidi = OxmlElement('w:bidi')
    bidi.set(qn('w:val'), '1')
    pPr.append(bidi)

def set_run_font(run, font_name="Cairo", size_pt=10, bold=False, italic=False, color_rgb=(15, 23, 42)):
    run.font.name = font_name
    run.font.size = Pt(size_pt)
    run.bold = bold
    run.italic = italic
    run.font.color.rgb = RGBColor(*color_rgb)
    
    # Enable complex script font for Arabic
    rPr = run._r.get_or_add_rPr()
    rFonts = rPr.find(qn('w:rFonts'))
    if rFonts is None:
        rFonts = OxmlElement('w:rFonts')
        rPr.append(rFonts)
    rFonts.set(qn('w:ascii'), font_name)
    rFonts.set(qn('w:hAnsi'), font_name)
    rFonts.set(qn('w:cs'), font_name)
    
    rtl = OxmlElement('w:rtl')
    rtl.set(qn('w:val'), '1')
    rPr.append(rtl)

def make_callout(doc, paragraphs_data, border_color="2563EB", bg_color="F8FAFC"):
    table = doc.add_table(rows=1, cols=1)
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    
    # RTL for table
    tblPr = table._tbl.tblPr
    bidiVisual = OxmlElement('w:bidiVisual')
    tblPr.append(bidiVisual)
    
    cell = table.cell(0, 0)
    cell.width = Cm(16.5)
    set_cell_background(cell, bg_color)
    set_cell_margins(cell, top=120, bottom=120, left=180, right=180)
    
    tcPr = cell._tc.get_or_add_tcPr()
    borders_xml = parse_xml(f'''
        <w:tcBorders {nsdecls("w")}>
            <w:top w:val="none"/>
            <w:left w:val="none"/>
            <w:bottom w:val="none"/>
            <w:right w:val="single" w:sz="24" w:space="0" w:color="{border_color}"/>
        </w:tcBorders>
    ''')
    tcPr.append(borders_xml)
    
    for i, p_data in enumerate(paragraphs_data):
        if i == 0:
            p = cell.paragraphs[0]
            p.paragraph_format.space_before = Pt(0)
            p.paragraph_format.space_after = Pt(2)
        else:
            p = cell.add_paragraph()
            p.paragraph_format.space_before = Pt(3)
            p.paragraph_format.space_after = Pt(2)
        p.alignment = WD_ALIGN_PARAGRAPH.RIGHT
        set_paragraph_rtl(p)
        
        for run_info in p_data:
            text = run_info[0]
            bold = run_info[1]
            color = run_info[2]
            run = p.add_run(text)
            set_run_font(run, font_name="Cairo", size_pt=9.5, bold=bold, color_rgb=color)

def build_document():
    doc = Document()
    
    # Page Setup (A4, 2cm margins)
    for section in doc.sections:
        section.page_width = Cm(21.0)
        section.page_height = Cm(29.7)
        section.top_margin = Cm(1.8)
        section.bottom_margin = Cm(1.8)
        section.left_margin = Cm(1.8)
        section.right_margin = Cm(1.8)
        
        # Header / Footer setup
        footer = section.footer
        f_p = footer.paragraphs[0]
        f_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        set_paragraph_rtl(f_p)
        f_run = f_p.add_run("المملكة الأردنية الهاشمية — جامعة جدارا | كلية تكنولوجيا المعلومات (قسم الروبوتات والذكاء الاصطناعي)")
        set_run_font(f_run, font_name="Cairo", size_pt=8, bold=False, color_rgb=(100, 116, 139))
    
    # Header Table
    h_table = doc.add_table(rows=1, cols=2)
    h_table.alignment = WD_TABLE_ALIGNMENT.CENTER
    h_table.autofit = False
    
    tblPr = h_table._tbl.tblPr
    bidiVisual = OxmlElement('w:bidiVisual')
    tblPr.append(bidiVisual)
    
    cell_right = h_table.cell(0, 0)
    cell_right.width = Cm(11.5)
    p_r = cell_right.paragraphs[0]
    p_r.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    set_paragraph_rtl(p_r)
    p_r.paragraph_format.space_before = Pt(0)
    p_r.paragraph_format.space_after = Pt(2)
    
    r1 = p_r.add_run("جامعة جدارا — Jadara University\n")
    set_run_font(r1, font_name="Cairo", size_pt=12, bold=True, color_rgb=(30, 58, 138))
    r2 = p_r.add_run("كلية تكنولوجيا المعلومات (Faculty of Information Technology)\n")
    set_run_font(r2, font_name="Cairo", size_pt=10, bold=True, color_rgb=(51, 65, 85))
    r3 = p_r.add_run("قسم علم الروبوتات والذكاء الاصطناعي (Robotics & AI Department)")
    set_run_font(r3, font_name="Cairo", size_pt=9, bold=False, color_rgb=(100, 116, 139))
    
    cell_left = h_table.cell(0, 1)
    cell_left.width = Cm(5.0)
    set_cell_background(cell_left, "EFF6FF")
    set_cell_margins(cell_left, top=80, bottom=80, left=100, right=100)
    set_cell_borders(cell_left, 
                     top={"sz": "4", "color": "BFDBFE"},
                     bottom={"sz": "4", "color": "BFDBFE"},
                     left={"sz": "4", "color": "BFDBFE"},
                     right={"sz": "4", "color": "BFDBFE"})
    
    p_l = cell_left.paragraphs[0]
    p_l.alignment = WD_ALIGN_PARAGRAPH.CENTER
    set_paragraph_rtl(p_l)
    p_l.paragraph_format.space_before = Pt(0)
    p_l.paragraph_format.space_after = Pt(0)
    
    b_run1 = p_l.add_run("وثيقة مشاركة رسمية\n")
    set_run_font(b_run1, font_name="Cairo", size_pt=9.5, bold=True, color_rgb=(29, 78, 216))
    b_run2 = p_l.add_run("مسابقة MMRC26 الوطنية\n")
    set_run_font(b_run2, font_name="Cairo", size_pt=8.5, bold=True, color_rgb=(30, 58, 138))
    b_run3 = p_l.add_run("التاريخ: 21 أيلول 2026")
    set_run_font(b_run3, font_name="Cairo", size_pt=8, bold=False, color_rgb=(100, 116, 139))
    
    # Horizontal separator
    doc.add_paragraph().paragraph_format.space_after = Pt(2)
    sep_p = doc.add_paragraph()
    sep_p.paragraph_format.space_before = Pt(0)
    sep_p.paragraph_format.space_after = Pt(6)
    pBdr = parse_xml(f'<w:pBdr {nsdecls("w")}><w:bottom w:val="single" w:sz="18" w:space="1" w:color="1E3A8A"/></w:pBdr>')
    sep_p._p.get_or_add_pPr().append(pBdr)
    
    # Title Box
    t_box = doc.add_table(rows=1, cols=1)
    t_box.alignment = WD_TABLE_ALIGNMENT.CENTER
    t_box.autofit = False
    tblPr = t_box._tbl.tblPr
    tblPr.append(OxmlElement('w:bidiVisual'))
    
    t_cell = t_box.cell(0, 0)
    t_cell.width = Cm(16.5)
    set_cell_background(t_cell, "1E3A8A")
    set_cell_margins(t_cell, top=140, bottom=140, left=180, right=180)
    
    tp = t_cell.paragraphs[0]
    tp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    set_paragraph_rtl(tp)
    tr1 = tp.add_run("طلب اعتماد ومشاركة رسمية في مسابقة حل المتاهة الوطنية للميكروماوس\n")
    set_run_font(tr1, font_name="Cairo", size_pt=12.5, bold=True, color_rgb=(255, 255, 255))
    tr2 = tp.add_run("Micromouse Maze Solver Contest 2026 (MMRC26) — IEEE RAS HTU")
    set_run_font(tr2, font_name="Cairo", size_pt=9.5, bold=False, color_rgb=(219, 234, 254))
    
    # Add Space
    sp = doc.add_paragraph()
    sp.paragraph_format.space_before = Pt(4)
    sp.paragraph_format.space_after = Pt(4)
    
    # Salutation
    sal_p = doc.add_paragraph()
    sal_p.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    set_paragraph_rtl(sal_p)
    sal_p.paragraph_format.space_before = Pt(2)
    sal_p.paragraph_format.space_after = Pt(4)
    s_run1 = sal_p.add_run("إلى: الدكتورة المشرفة المحترمة\n")
    set_run_font(s_run1, font_name="Cairo", size_pt=10.5, bold=True, color_rgb=(30, 58, 138))
    s_run2 = sal_p.add_run("تحية طيبة وبعد،،،\nيرجى التكرم بالاطلاع على التقرير الفني والتنظيمي المرفق أدناه، والمتضمن كافة تفاصيل مسابقة الميكروماوس الوطنية لحل المتاهة (MMRC26)، وموعدها، ومكان انعقادها، وبيانات الفريق الممثل لقسم الروبوتات والذكاء الاصطناعي بكلية تكنولوجيا المعلومات، بالإضافة إلى الكشف الهندسي الشامل لجميع قطع ومكونات الروبوت (القطع المعتمدة والقطع الجديدة المضافة) لاعتماد الكتاب الرسمي وتوجيهه للجهات المعنية.")
    set_run_font(s_run2, font_name="Cairo", size_pt=9.5, bold=False, color_rgb=(51, 65, 85))
    
    # Helper to add section heading
    def add_section_heading(title_text):
        p = doc.add_paragraph()
        p.alignment = WD_ALIGN_PARAGRAPH.RIGHT
        set_paragraph_rtl(p)
        p.paragraph_format.space_before = Pt(10)
        p.paragraph_format.space_after = Pt(4)
        
        run = p.add_run(title_text)
        set_run_font(run, font_name="Cairo", size_pt=11, bold=True, color_rgb=(30, 58, 138))
        
        pPr = p._p.get_or_add_pPr()
        pBdr = parse_xml(f'<w:pBdr {nsdecls("w")}><w:bottom w:val="single" w:sz="8" w:space="2" w:color="2563EB"/></w:pBdr>')
        pPr.append(pBdr)

    # 1. وصف ونبذة المسابقة
    add_section_heading("أولاً: وصف ونبذة عن المسابقة (Competition Overview)")
    
    desc_points = [
        [("• اسم المسابقة الرسمي: ", True, (30, 58, 138)), 
         ("مسابقة الميكروماوس لحل المتاهة 2026 (Micromouse Maze Solver Contest – MMRC26).", False, (15, 23, 42))],
        [("• الجهة المنظمة: ", True, (30, 58, 138)), 
         ("فرع جمعية الروبوتات والأتمتة التابع لمعهد مهندسي الكهرباء والإلكترونيات (IEEE RAS Student Branch Chapter) في جامعة الحسين التقنية (HTU).", False, (15, 23, 42))],
        [("• طبيعة التحدي وأهدافه: ", True, (30, 58, 138)), 
         ("تحدٍ وطني وهندسي قياسي في مجال الروبوتات والذكاء الاصطناعي المدمج، يتطلب تصميم وبناء وبرمجة روبوت متحرك مستقل بالكامل (Fully Autonomous Micromouse). يبدأ الروبوت من زاوية متاهة قياسية بأبعاد (10 × 10) مربعات لاستكشاف المسار ذاتياً دون أي تدخل بشري أو اتصال خارجي، والوصول إلى الهدف المركزي المعزول (Island Goal) بأقصر زمن ممكن وبأعلى عدد من المحاولات الناجحة خلال نافذة زمنية مدتها 8 دقائق.", False, (15, 23, 42))],
        [("• المعيار الرياضي للتقييم: ", True, (30, 58, 138)), 
         ("يعتمد الترتيب الرسمي على معادلة النقاط التراكمية: Final Score = (Total Successful Runs / Official Fastest Time) * 1000، مما يتطلب توازناً ذكياً بين استكشاف المتاهة وسرعة الوصول ودقة التكرار.", False, (15, 23, 42))]
    ]
    make_callout(doc, desc_points, border_color="1E3A8A", bg_color="F8FAFC")
    
    # 2. الموعد والمكان
    add_section_heading("ثانياً: موعد ومكان انعقاد المسابقة (Date & Location)")
    
    loc_table = doc.add_table(rows=4, cols=2)
    loc_table.alignment = WD_TABLE_ALIGNMENT.CENTER
    loc_table.autofit = False
    tblPr = loc_table._tbl.tblPr
    tblPr.append(OxmlElement('w:bidiVisual'))
    
    loc_data = [
        ("تاريخ المسابقة المعتمد", "الإثنين، 5 تشرين الأول (أكتوبر) 2026 (Monday, 5 October 2026)"),
        ("الموقع والقاعة", "مبنى The ARC — حرم جامعة الحسين التقنية (Al-Hussein Technical University)"),
        ("العنوان الجغرافي", "مجمع الملك الحسين للأعمال (King Hussein Business Park - KHBP)، شارع الملك عبدالله الثاني، عمّان"),
        ("فترة الفحص والتسجيل", "من الساعة 08:00 صباحاً وحتى الساعة 09:00 صباحاً (فحص الهاردوير والتأكيد الإلزامي)")
    ]
    
    for row_idx, (title, val) in enumerate(loc_data):
        c0 = loc_table.cell(row_idx, 0)
        c1 = loc_table.cell(row_idx, 1)
        c0.width = Cm(4.5)
        c1.width = Cm(12.0)
        
        bg = "EFF6FF" if row_idx % 2 == 0 else "FFFFFF"
        set_cell_background(c0, bg)
        set_cell_background(c1, bg)
        
        set_cell_margins(c0, top=70, bottom=70, left=100, right=100)
        set_cell_margins(c1, top=70, bottom=70, left=100, right=100)
        
        set_cell_borders(c0, bottom={"sz": "4", "color": "E2E8F0"}, right={"sz": "4", "color": "E2E8F0"})
        set_cell_borders(c1, bottom={"sz": "4", "color": "E2E8F0"}, left={"sz": "4", "color": "E2E8F0"})
        
        p0 = c0.paragraphs[0]
        p0.alignment = WD_ALIGN_PARAGRAPH.RIGHT
        set_paragraph_rtl(p0)
        r0 = p0.add_run(title)
        set_run_font(r0, font_name="Cairo", size_pt=9, bold=True, color_rgb=(30, 58, 138))
        
        p1 = c1.paragraphs[0]
        p1.alignment = WD_ALIGN_PARAGRAPH.RIGHT
        set_paragraph_rtl(p1)
        r1 = p1.add_run(val)
        set_run_font(r1, font_name="Cairo", size_pt=9, bold=False, color_rgb=(15, 23, 42))

    # 3. الجدول الزمني التفصيلي
    add_section_heading("ثالثاً: البرنامج الزمني الرسمي ليوم الفعالية (Official Timeline)")
    
    sched_table = doc.add_table(rows=9, cols=3)
    sched_table.alignment = WD_TABLE_ALIGNMENT.CENTER
    sched_table.autofit = False
    sched_table._tbl.tblPr.append(OxmlElement('w:bidiVisual'))
    
    headers = ["الوقت", "النشاط / الفعالية الرسمية", "المكان داخل الحرم"]
    for col_idx, h in enumerate(headers):
        c = sched_table.cell(0, col_idx)
        set_cell_background(c, "1E3A8A")
        set_cell_margins(c, top=80, bottom=80, left=80, right=80)
        p = c.paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        set_paragraph_rtl(p)
        r = p.add_run(h)
        set_run_font(r, font_name="Cairo", size_pt=9, bold=True, color_rgb=(255, 255, 255))
        
    sched_table.cell(0, 0).width = Cm(3.2)
    sched_table.cell(0, 1).width = Cm(9.8)
    sched_table.cell(0, 2).width = Cm(3.5)
    
    schedule_data = [
        ("08:00 - 09:00", "تسجيل الفرق، استلام البطاقات، وفحص الهاردوير الإلزامي (Hardware Inspection)", "Registration Desk"),
        ("09:00 - 09:30", "الجلسة الافتتاحية وإعلان قرعة وترتيب جولات التصفيات (Opening Briefing)", "Main Arena"),
        ("09:30 - 13:30", "المرحلة الأولى: تصفيات المتاهة الفردية لكافة الفرق (Phase 1: Single-Maze Qualifiers)", "Main Arena"),
        ("13:30 - 14:30", "استراحة الغداء وإعلان قائمة الفرق المتأهلة للأدوار الإقصائية (Cutoff Announcement)", "Campus / Pit Area"),
        ("14:30 - 17:30", "المرحلة الثانية: دوري خروج المغلوب والمواجهات المزدوجة المتزامنة (Phase 2: Knockout League)", "Main Arena"),
        ("17:30 - 18:00", "الحفل الختامي وتوزيع الجوائز التقنية المتخصصة (Best Code & Creative Design)", "Main Arena"),
        ("18:00 - 18:20", "المواجهة الختامية الكبرى لتحديد بطل المسابقة (The Grand Final Match)", "Main Arena"),
        ("18:20 - 18:45", "إعلان الفائزين رسمياً، تتويج الأبطال وتسليم الكؤوس والشهادات (Trophy Presentation)", "Main Arena")
    ]
    
    for row_idx, (t, act, loc) in enumerate(schedule_data, start=1):
        bg = "F8FAFC" if row_idx % 2 == 0 else "FFFFFF"
        for col_idx, text in enumerate([t, act, loc]):
            c = sched_table.cell(row_idx, col_idx)
            set_cell_background(c, bg)
            set_cell_margins(c, top=60, bottom=60, left=80, right=80)
            set_cell_borders(c, bottom={"sz": "4", "color": "E2E8F0"}, 
                             left={"sz": "4", "color": "E2E8F0"}, 
                             right={"sz": "4", "color": "E2E8F0"})
            p = c.paragraphs[0]
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER if col_idx != 1 else WD_ALIGN_PARAGRAPH.RIGHT
            set_paragraph_rtl(p)
            r = p.add_run(text)
            set_run_font(r, font_name="Cairo", size_pt=8.5, bold=(col_idx == 0), color_rgb=(15, 23, 42))

    # 4. بيانات الفريق المشارك
    add_section_heading("رابعاً: بيانات فريق العمل الممثل للجامعة (Team Roster)")
    
    team_table = doc.add_table(rows=4, cols=4)
    team_table.alignment = WD_TABLE_ALIGNMENT.CENTER
    team_table.autofit = False
    team_table._tbl.tblPr.append(OxmlElement('w:bidiVisual'))
    
    t_headers = ["الصفة في الفريق", "الاسم الكامل", "الكلية", "التخصص الأكاديمي"]
    for col_idx, h in enumerate(t_headers):
        c = team_table.cell(0, col_idx)
        set_cell_background(c, "2563EB")
        set_cell_margins(c, top=70, bottom=70, left=70, right=70)
        p = c.paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        set_paragraph_rtl(p)
        r = p.add_run(h)
        set_run_font(r, font_name="Cairo", size_pt=9, bold=True, color_rgb=(255, 255, 255))
        
    team_table.cell(0, 0).width = Cm(3.5)
    team_table.cell(0, 1).width = Cm(5.5)
    team_table.cell(0, 2).width = Cm(3.5)
    team_table.cell(0, 3).width = Cm(4.0)
    
    team_members = [
        ("قائد الفريق ومطور الأنظمة", "كرم احمد عصام خصاونه", "تكنولوجيا المعلومات (IT)", "علم الروبوتات والذكاء الاصطناعي"),
        ("عضو الفريق (مشارك)", "انور شطناوي", "تكنولوجيا المعلومات (IT)", "علم الروبوتات والذكاء الاصطناعي"),
        ("عضو الفريق (مشارك)", "عبدالرحمان العكور", "تكنولوجيا المعلومات (IT)", "علم الروبوتات والذكاء الاصطناعي")
    ]
    
    for row_idx, mem in enumerate(team_members, start=1):
        bg = "EFF6FF" if row_idx == 1 else "FFFFFF"
        for col_idx, val in enumerate(mem):
            c = team_table.cell(row_idx, col_idx)
            set_cell_background(c, bg)
            set_cell_margins(c, top=65, bottom=65, left=70, right=70)
            set_cell_borders(c, bottom={"sz": "4", "color": "CBD5E1"}, 
                             left={"sz": "4", "color": "CBD5E1"}, 
                             right={"sz": "4", "color": "CBD5E1"})
            p = c.paragraphs[0]
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER
            set_paragraph_rtl(p)
            is_bold = (col_idx <= 1 and row_idx == 1)
            color = (30, 58, 138) if (row_idx == 1 and col_idx == 1) else (15, 23, 42)
            r = p.add_run(val)
            set_run_font(r, font_name="Cairo", size_pt=8.5, bold=is_bold, color_rgb=color)

    # Note about team capacity
    note_p = doc.add_paragraph()
    note_p.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    set_paragraph_rtl(note_p)
    note_p.paragraph_format.space_before = Pt(3)
    note_p.paragraph_format.space_after = Pt(6)
    nr = note_p.add_run("* ملاحظة: تنص المادة (3.b) من القوانين الرسمية على أن الفريق يتكون من طالب واحد إلى 3 طلاب كحد أقصى. المشرف الأكاديمي: الدكتورة المشرفة المحترمة.")
    set_run_font(nr, font_name="Cairo", size_pt=8, italic=True, color_rgb=(100, 116, 139))

    # 5. المتطلبات والمعايير الفنية
    add_section_heading("خامساً: المتطلبات والقواعد الفنية الإلزامية للمسابقة (Rules & Constraints)")
    
    rules_text = [
        [("1. استقلالية الروبوت التامة (Self-Contained): ", True, (30, 58, 138)),
         ("يجب أن يعمل الروبوت كلياً بالمعالجات والبطاريات الداخلية. يُحظر تماماً أي تحكم عن بعد أو اتصال لاسلكي (Wi-Fi/Bluetooth/RF) أو اتصال سلكي خارجي.", False, (15, 23, 42))],
        [("2. الأبعاد الهندسية الصارمة: ", True, (30, 58, 138)),
         ("يجب ألا تتجاوز أبعاد الروبوت 25 سم طولاً × 25 سم عرضاً في أي لحظة من زمن الجولة (لا قيود على الارتفاع).", False, (15, 23, 42))],
        [("3. طبيعة المتاهة المعقدة (Island Goal): ", True, (30, 58, 138)),
         ("تتكون المتاهة من 10 × 10 مربعات (مقاس كل مربع 18 × 18 سم)، والمركز معزول عن الجدران الخارجية تماماً، مما يُفشل خوارزميات تتبع الجدران التقليدية (Wall-Hugging) ويتطلب إلزامياً خوارزميات ذكاء اصطناعي وبحث متقدمة (Modified Flood Fill / A*).", False, (15, 23, 42))],
        [("4. التدقيق البرمجي والنزاهة الأكاديمية (Source Code Audit): ", True, (30, 58, 138)),
         ("يُمنع إدخال بيانات المتاهة مسبقاً، وتخضع جميع الأكواد المصدرية للفحص الدقيق من قبل لجنة التحكيم لضمان تشغيل خوارزميات الملاحة الحية المستقلة.", False, (15, 23, 42))]
    ]
    make_callout(doc, rules_text, border_color="2563EB", bg_color="F8FAFC")

    # 6. كشف القطع الهندسية الشامل (القطع الأصلية والقطع الجديدة)
    add_section_heading("سادساً: كشف القطع والمكونات الهندسية الشامل للروبوت (Hardware Bill of Materials)")
    
    bom_intro = doc.add_paragraph()
    bom_intro.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    set_paragraph_rtl(bom_intro)
    bom_intro.paragraph_format.space_before = Pt(2)
    bom_intro.paragraph_format.space_after = Pt(4)
    bi_run = bom_intro.add_run("يشمل الكشف أدناه كافة القطع الـ 14 المعتمدة مسبقاً بالإضافة إلى المكونات والقطع الجديدة المضافة (وحدة المعالجة المركزية الذكية ESP32، ومنظومة حساسات هول المغناطيسية لحساب المسافة بدقة عالية، وأزرار التشغيل، وملحقات التوصيل):")
    set_run_font(bi_run, font_name="Cairo", size_pt=8.5, bold=False, color_rgb=(51, 65, 85))
    
    bom_table = doc.add_table(rows=18, cols=6)
    bom_table.alignment = WD_TABLE_ALIGNMENT.CENTER
    bom_table.autofit = False
    bom_table._tbl.tblPr.append(OxmlElement('w:bidiVisual'))
    
    bom_headers = ["#", "اسم القطعة كما في السلة (Item Description)", "الوظيفة الهندسية في منظومة الروبوت", "الكمية", "السعر الإفرادي", "الإجمالي (د.أ)"]
    for col_idx, h in enumerate(bom_headers):
        c = bom_table.cell(0, col_idx)
        set_cell_background(c, "1E3A8A")
        set_cell_margins(c, top=70, bottom=70, left=50, right=50)
        p = c.paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        set_paragraph_rtl(p)
        r = p.add_run(h)
        set_run_font(r, font_name="Cairo", size_pt=8.0, bold=True, color_rgb=(255, 255, 255))
        
    bom_table.cell(0, 0).width = Cm(0.7)
    bom_table.cell(0, 1).width = Cm(5.2)
    bom_table.cell(0, 2).width = Cm(5.4)
    bom_table.cell(0, 3).width = Cm(1.2)
    bom_table.cell(0, 4).width = Cm(2.0)
    bom_table.cell(0, 5).width = Cm(2.0)
    
    # Exactly matching the official shopping cart screenshot (16 items)
    bom_items = [
        (1, "CJMCU-530 Laser Ranging Sensor Module Rangefinder", "حساسات مسافة ليزرية ToF بدقة ملليمترية لكشف الجدران والتوسيط وتفادي الصدمات", 3, "9.900", "29.700"),
        (2, "MINI 3PI car N20 Caster Robot Ball Wheel", "عجلة ارتكاز كروية خلفية حرة الحركة لمنح الروبوت توازناً وانسيابية في الالتفاف", 1, "1.450", "1.450"),
        (3, "Motor Mount Bracket for N20 Motor 3PI miniQ", "قواعد تثبيت هندسية صلبة لتأمين استقامة وصلابة المحركات على الشاسيه", 2, "0.650", "1.300"),
        (4, "N20 6V 500 RPM HIGH TORQUE DC MOTOR", "محركات الدفع الرئيسية بعزم وسرعة مدروسة لتنفيذ استكشاف وسبرنت المتاهة", 2, "5.400", "10.800"),
        (5, "6-Axis Accelerometer Gyroscope MPU6050 - GY521", "جيروسكوب لتثبيت زاوية التوجيه (Heading PID) والانعطاف 90°/180° بدقة", 1, "5.000", "5.000"),
        (6, "Rubber Wheel and Tire 34mm for N20 (D-hole)", "عجلات مطاطية عالية التماسك مخصصة لمحاور D-shaft لمنع الانزلاق", 2, "1.200", "2.400"),
        (7, "Mini DC-DC Step Down Buck Converter Module", "خافض جهد عالي الكفاءة لتحويل 7.4V إلى 5V مستقرة لحماية وتغذية المعالج", 2, "1.950", "3.900"),
        (8, "SINGLE SIDE PROTOTYPE PCB 7x9 CM", "لوحات تخريم لتجميع الدارات وبناء شاسيه الروبوت بأبعاد مدمجة مطابقة للقوانين", 4, "0.400", "1.600"),
        (9, "Breakable straight 1x40pin Male Headers 2.54mm", "رؤوس توصيل قياسية لتسهيل فك وتركيب وتبديل الحساسات لأغراض الصيانة", 4, "0.150", "0.600"),
        (10, "Solder Wire PRO'SKIT 9S001 17g 1mm", "سلك قصدير لحام إلكتروني عالي الجودة لضمان توصيل كهربائي متين وموثوق", 1, "2.550", "2.550"),
        (11, "Pro'sKit Soldering Iron 40W 8PK-S118B-40W", "كاوية لحام إلكترونية بقدرة 40W لتجميع الدارات والأسلاك وتثبيت المكونات", 1, "6.000", "6.000"),
        (12, "2 Channel MX1508 DC Motor Drive Module PWM", "درايفر قيادة المحركات بتقنية MOSFET المدمجة بتيار 1.5A وتوافق مع 3.3V Logic", 1, "1.900", "1.900"),
        (13, "Dual Charger For 18650 3.7V Li-ion Battery", "شاحن جداري مخصص لشحن بطاريات الليثيوم 18650 بأمان لاستمرارية التجارب", 1, "6.000", "6.000"),
        (14, "BATTERY HOLDER 2X18650 3.7V", "حامل بطاريتين 18650 موصول توالي لتوليد 7.4V لتغذية المحركات والدرايفر", 1, "1.000", "1.000"),
        (15, "Hall effect Sensor Module", "حساسات هول مغناطيسية على العجلات تعمل كإنكودر دقيق لقياس المسافة واجتياز الخلايا", 2, "1.500", "3.000"),
        (16, "magnet 3x5 mm cylinder", "مغانط أسطوانية مصغرة تثبت على العجلات لتفعيل حساسات هول بدقة وسرعة", 10, "0.250", "2.500")
    ]
    
    for row_idx, item in enumerate(bom_items, start=1):
        is_new = (item[0] >= 15)
        bg = "EFF6FF" if is_new else ("F8FAFC" if row_idx % 2 == 0 else "FFFFFF")
        
        for col_idx, val in enumerate(item):
            c = bom_table.cell(row_idx, col_idx)
            set_cell_background(c, bg)
            set_cell_margins(c, top=45, bottom=45, left=50, right=50)
            set_cell_borders(c, bottom={"sz": "4", "color": "CBD5E1"}, 
                             left={"sz": "4", "color": "CBD5E1"}, 
                             right={"sz": "4", "color": "CBD5E1"})
            p = c.paragraphs[0]
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER if col_idx in [0, 3, 4, 5] else WD_ALIGN_PARAGRAPH.RIGHT
            set_paragraph_rtl(p)
            
            font_size = 7.5
            bold_val = (col_idx in [0, 5] or (is_new and col_idx == 1))
            text_color = (29, 78, 216) if (is_new and col_idx == 1) else (15, 23, 42)
            
            r = p.add_run(str(val))
            set_run_font(r, font_name="Cairo", size_pt=font_size, bold=bold_val, color_rgb=text_color)
            
    # Total Row (Row 17)
    tot_row = bom_table.cell(17, 0)
    for i in range(1, 5):
        tot_row.merge(bom_table.cell(17, i))
    set_cell_background(tot_row, "EFF6FF")
    p_tot = tot_row.paragraphs[0]
    p_tot.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    set_paragraph_rtl(p_tot)
    r_tot_txt = p_tot.add_run("المجموع الإجمالي الكلي المعتمد لسلة الشراء (Sub Total):")
    set_run_font(r_tot_txt, font_name="Cairo", size_pt=8.5, bold=True, color_rgb=(30, 58, 138))
    
    tot_val_cell = bom_table.cell(17, 5)
    set_cell_background(tot_val_cell, "EFF6FF")
    p_v = tot_val_cell.paragraphs[0]
    p_v.alignment = WD_ALIGN_PARAGRAPH.CENTER
    set_paragraph_rtl(p_v)
    r_v = p_v.add_run("79.700 د.أ")
    set_run_font(r_v, font_name="Cairo", size_pt=9.5, bold=True, color_rgb=(29, 78, 216))

    # 7. المبررات الهندسية والتقنية
    add_section_heading("سابعاً: المبررات الهندسية والتقنية لاعتماد المنظومة (Technical Defense)")
    
    just_points = [
        [("1. الالتزام بالأبعاد وقوانين الحلبة: ", True, (30, 58, 138)),
         ("الأبعاد المجمعة للروبوت (9 سم طولاً × 7.5 سم عرضاً) أصغر بكثير من الحد الأقصى المسموح (25 × 25 سم)، مما يضمن التفافاً انسيابياً بنصف قطر 3.5 سم داخل خلايا المتاهة (18 × 18 سم) دون أي احتكاك بالجدران.", False, (15, 23, 42))],
        [("2. منظومة التحكم المغلقة الهجينة (Hybrid Closed-Loop Odometry): ", True, (30, 58, 138)),
         ("دمج جيروسكوب MPU6050 لضبط زوايا الانعطاف (Heading PID 90°/180°)، مع حساسات الليزر الثلاثية VL53L0X لتوسيط الروبوت، بالإضافة لحساسات هول A3144 لحساب مسافة اجتياز الخلايا بشكل قطعي يمنع تراكم أخطاء الإزاحة.", False, (15, 23, 42))],
        [("3. العزل الكهربائي وتوزيع القدرة (Power Architecture): ", True, (30, 58, 138)),
         ("فصل دائرة تغذية المحركات (7.4V عبر بطاريات 18650) عن دائرة تغذية معالج ESP32 (5V عبر Buck Converter) لمنع هبوط الجهد (Voltage Dips) وحدوث إعادة تشغيل مفاجئة (Brownout Reset) أثناء تسارع المحركات.", False, (15, 23, 42))],
        [("4. الكفاءة البرمجية المتقدمة: ", True, (30, 58, 138)),
         ("خوارزمية الاستكشاف تعتمد Modified Flood Fill، وخوارزمية سبرنت السرعة تعتمد Turn-Weighted A* المبنية بتقنية الذاكرة الصفرية (Zero-Allocation C99) التي تستهلك أقل من 50 بايت من الذاكرة لضمان سرعة استجابة فورية بدون أي تأخير زمني.", False, (15, 23, 42))]
    ]
    make_callout(doc, just_points, border_color="2563EB", bg_color="F8FAFC")

    # 8. التنسيب والتواقيع الرسمية
    add_section_heading("ثامناً: التنسيب والتواقيع الرسمية (Official Signatures)")
    
    sig_table = doc.add_table(rows=1, cols=3)
    sig_table.alignment = WD_TABLE_ALIGNMENT.CENTER
    sig_table.autofit = False
    sig_table._tbl.tblPr.append(OxmlElement('w:bidiVisual'))
    
    sig_cols = [
        ("قائد فريق العمل", "كرم احمد عصام خصاونه", "التوقيع: ........................\nالتاريخ: 21 / 09 / 2026"),
        ("مشرف المشروع الأكاديمي", "الدكتورة المشرفة المحترمة", "الموافقة والتنسيب: ............\nالتاريخ: .... / .... / 2026"),
        ("عمادة كلية تكنولوجيا المعلومات", "جامعة جدارا", "الختم الرسمي والاعتماد: ......\nالتاريخ: .... / .... / 2026")
    ]
    
    for col_idx, (role, name, sig) in enumerate(sig_cols):
        c = sig_table.cell(0, col_idx)
        c.width = Cm(5.5)
        set_cell_background(c, "F8FAFC")
        set_cell_margins(c, top=100, bottom=100, left=100, right=100)
        set_cell_borders(c, top={"sz": "4", "color": "CBD5E1"},
                         bottom={"sz": "4", "color": "CBD5E1"},
                         left={"sz": "4", "color": "CBD5E1"},
                         right={"sz": "4", "color": "CBD5E1"})
        p = c.paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        set_paragraph_rtl(p)
        
        r1 = p.add_run(f"{role}\n")
        set_run_font(r1, font_name="Cairo", size_pt=9.5, bold=True, color_rgb=(30, 58, 138))
        r2 = p.add_run(f"{name}\n\n")
        set_run_font(r2, font_name="Cairo", size_pt=9, bold=False, color_rgb=(15, 23, 42))
        r3 = p.add_run(sig)
        set_run_font(r3, font_name="Cairo", size_pt=8, bold=False, color_rgb=(100, 116, 139))

    output_path = r"c:\Users\ASUS\OneDrive\المستندات\GitHub\Maze-Solver\MMRC26_Official_Participation_Letter.docx"
    doc.save(output_path)
    print(f"Document saved successfully at: {output_path}")

if __name__ == "__main__":
    build_document()
