import os
from playwright.sync_api import sync_playwright

html_content = """<!DOCTYPE html>
<html lang="ar" dir="rtl">
<head>
<meta charset="UTF-8">
<style>
    @import url('https://fonts.googleapis.com/css2?family=Cairo:wght@400;600;700;800&display=swap');
    
    @page {
        size: A4 portrait;
        margin: 8mm 10mm 8mm 10mm;
    }

    body {
        font-family: 'Cairo', 'Segoe UI', Arial, sans-serif;
        color: #0f172a;
        margin: 0;
        padding: 0;
        line-height: 1.35;
        font-size: 8.5pt;
        background-color: #ffffff;
    }

    .header-table {
        width: 100%;
        border-bottom: 2.5px solid #1e3a8a;
        padding-bottom: 6px;
        margin-bottom: 8px;
    }

    .header-table td {
        vertical-align: middle;
    }

    .univ-title {
        font-size: 13pt;
        font-weight: 800;
        color: #1e3a8a;
        margin: 0;
    }

    .faculty-title {
        font-size: 9.5pt;
        font-weight: 700;
        color: #334155;
        margin: 1px 0;
    }

    .doc-badge {
        background-color: #eff6ff;
        border: 1px solid #bfdbfe;
        color: #1d4ed8;
        padding: 4px 10px;
        border-radius: 5px;
        font-size: 8pt;
        font-weight: 700;
        display: inline-block;
        text-align: left;
        line-height: 1.3;
    }

    .title-banner {
        background: linear-gradient(135deg, #1e3a8a 0%, #2563eb 100%);
        color: white;
        padding: 7px 12px;
        border-radius: 6px;
        margin-bottom: 10px;
    }

    .title-banner h1 {
        margin: 0;
        font-size: 11.5pt;
        font-weight: 800;
    }

    .title-banner .sub {
        font-size: 8pt;
        opacity: 0.95;
        margin-top: 1px;
    }

    .section-title {
        font-size: 9.5pt;
        font-weight: 700;
        color: #0f172a;
        border-right: 3px solid #2563eb;
        padding-right: 6px;
        margin: 8px 0 6px 0;
    }

    table.items-table {
        width: 100%;
        border-collapse: collapse;
        margin-bottom: 8px;
        font-size: 7.8pt;
    }

    table.items-table th {
        background-color: #f1f5f9;
        color: #0f172a;
        font-weight: 700;
        padding: 4px 5px;
        border: 1px solid #cbd5e1;
        text-align: center;
    }

    table.items-table td {
        padding: 3.5px 5px;
        border: 1px solid #e2e8f0;
        vertical-align: middle;
    }

    table.items-table tr:nth-child(even) {
        background-color: #f8fafc;
    }

    .text-center { text-align: center; }
    .font-bold { font-weight: 700; }

    .price-col {
        font-family: 'Segoe UI', Arial, sans-serif;
        font-weight: 700;
        white-space: nowrap;
        text-align: center;
    }

    .total-row {
        background-color: #eff6ff !important;
        border-top: 2px solid #2563eb !important;
        font-weight: 800;
        font-size: 8.5pt;
    }

    .total-price {
        color: #1d4ed8;
        font-size: 9.5pt;
    }

    .justification-box {
        background-color: #f8fafc;
        border: 1px solid #e2e8f0;
        border-radius: 5px;
        padding: 7px 12px;
        margin-bottom: 8px;
        font-size: 7.6pt;
    }

    .justification-box ul {
        margin: 2px 0 0 0;
        padding-right: 16px;
    }

    .justification-box li {
        margin-bottom: 2px;
    }

    .footer-table {
        width: 100%;
        margin-top: 10px;
        border-top: 1px solid #cbd5e1;
        padding-top: 6px;
        font-size: 8pt;
    }

    .signature-box {
        border-top: 1px dashed #94a3b8;
        margin-top: 22px;
        padding-top: 3px;
        color: #64748b;
        font-size: 7.5pt;
        text-align: center;
    }
</style>
</head>
<body>

<table class="header-table">
    <tr>
        <td style="width: 70%;">
            <div class="univ-title">المملكة الأردنية الهاشمية — جامعة جدارا</div>
            <div class="faculty-title">كلية الهندسة — قسم هندسة الميكاترونكس والروبوتات والذكاء الاصطناعي</div>
            <div style="font-size: 8pt; color: #64748b;">
                مشاركة مسابقة: Micromouse Maze Solver Contest 2026 (MMRC26) — تنظيم IEEE RAS HTU
            </div>
        </td>
        <td style="width: 30%; text-align: left;">
            <div class="doc-badge">
                وثيقة اعتماد رسمية (BOM)<br>
                <strong>طلب شراء وتغطية مالية</strong><br>
                التاريخ: 21 أيلول 2026
            </div>
        </td>
    </tr>
</table>

<div class="title-banner">
    <h1>كشف المواد الفنية والميزانية المعتمدة لروبوت الميكروماوس الذاتي</h1>
    <div class="sub">مقدم إلى الدكتورة المشرفة المحترمة — مشروع تمثيل جامعة جدارا في البطولة الوطنية لحل المتاهة 2026</div>
</div>

<div class="section-title">أولاً: جدول بنود المواد والتكلفة الموثقة (Bill of Materials)</div>

<table class="items-table">
    <thead>
        <tr>
            <th style="width: 4%;">#</th>
            <th style="width: 34%;">اسم القطعة (Item Specification)</th>
            <th style="width: 34%;">الوظيفة الهندسية في منظومة الروبوت</th>
            <th style="width: 6%;">الكمية</th>
            <th style="width: 11%;">السعر الإفرادي</th>
            <th style="width: 11%;">الإجمالي (د.أ)</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td class="text-center font-bold">1</td>
            <td class="font-bold">CJMCU-530 Laser Sensor (VL53L0X)</td>
            <td>حساسات مسافة ليزرية بدقة ملليمترية (ToF) لكشف الجدران والتوسيط وتفادي الصدمات</td>
            <td class="text-center font-bold">3</td>
            <td class="price-col">9.900</td>
            <td class="price-col font-bold">29.700</td>
        </tr>
        <tr>
            <td class="text-center font-bold">2</td>
            <td class="font-bold">GA12-N20 6V 500RPM High Torque Motor</td>
            <td>محركات الدفع الرئيسية بعزم وسرعة مدروسة لتنفيذ استكشاف وسبرنت المتاهة</td>
            <td class="text-center font-bold">2</td>
            <td class="price-col">5.400</td>
            <td class="price-col font-bold">10.800</td>
        </tr>
        <tr>
            <td class="text-center font-bold">3</td>
            <td class="font-bold">Dual Charger For 18650 Li-Ion Battery</td>
            <td>شاحن جداري مخصص لشحن بطاريات الليثيوم 18650 بأمان لتأمين استمرارية التجارب</td>
            <td class="text-center font-bold">1</td>
            <td class="price-col">6.000</td>
            <td class="price-col font-bold">6.000</td>
        </tr>
        <tr>
            <td class="text-center font-bold">4</td>
            <td class="font-bold">Pro'sKit Soldering Iron 40W (8PK-S118B)</td>
            <td>كاوية لحام إلكترونية بقدرة 40W لتجميع الدارات والأسلاك وتثبيت المكونات</td>
            <td class="text-center font-bold">1</td>
            <td class="price-col">6.000</td>
            <td class="price-col font-bold">6.000</td>
        </tr>
        <tr>
            <td class="text-center font-bold">5</td>
            <td class="font-bold">6-Axis Gyroscope MPU6050 (GY-521)</td>
            <td>جيروسكوب لتثبيت التوجيه بخط مستقيم (Heading PID) وتنفيذ لفات 90°/180° بدقة</td>
            <td class="text-center font-bold">1</td>
            <td class="price-col">5.000</td>
            <td class="price-col font-bold">5.000</td>
        </tr>
        <tr>
            <td class="text-center font-bold">6</td>
            <td class="font-bold">Mini DC-DC Step Down Buck Converter</td>
            <td>خافض جهد عالي الكفاءة لتحويل 7.4V إلى 5V مستقرة لحماية وتغذية معالج ESP32</td>
            <td class="text-center font-bold">2</td>
            <td class="price-col">1.950</td>
            <td class="price-col font-bold">3.900</td>
        </tr>
        <tr>
            <td class="text-center font-bold">7</td>
            <td class="font-bold">Solder Wire PRO'SKIT 9S001 17g 1mm</td>
            <td>سلك قصدير لحام إلكتروني عالي الجودة لضمان توصيل كهربائي متين</td>
            <td class="text-center font-bold">1</td>
            <td class="price-col">2.550</td>
            <td class="price-col font-bold">2.550</td>
        </tr>
        <tr>
            <td class="text-center font-bold">8</td>
            <td class="font-bold">Rubber Wheel and Tire 34mm (D-hole)</td>
            <td>عجلات مطاطية عالية التماسك مخصصة لمحاور D-shaft لمحركات N20 لمنع الانزلاق</td>
            <td class="text-center font-bold">2</td>
            <td class="price-col">1.200</td>
            <td class="price-col font-bold">2.400</td>
        </tr>
        <tr>
            <td class="text-center font-bold">9</td>
            <td class="font-bold">MX1508 Dual DC Motor Driver (PWM)</td>
            <td>دارة قيادة المحركات بتقنية MOSFET المدمجة بتيار 1.5A وتوافق كامل مع 3.3V Logic</td>
            <td class="text-center font-bold">1</td>
            <td class="price-col">1.900</td>
            <td class="price-col font-bold">1.900</td>
        </tr>
        <tr>
            <td class="text-center font-bold">10</td>
            <td class="font-bold">Single Side Prototype PCB 7x9 CM</td>
            <td>لوحات تخريم لبناء وتجميع شاسيه الروبوت بأبعاد مدمجة مطابقة لقوانين المسابقة</td>
            <td class="text-center font-bold">4</td>
            <td class="price-col">0.400</td>
            <td class="price-col font-bold">1.600</td>
        </tr>
        <tr>
            <td class="text-center font-bold">11</td>
            <td class="font-bold">MINI 3PI N20 Caster Robot Ball Wheel</td>
            <td>عجلة ارتكاز كروية خلفية حرة الحركة لمنح الروبوت توازناً وانسيابية في الالتفاف</td>
            <td class="text-center font-bold">1</td>
            <td class="price-col">1.450</td>
            <td class="price-col font-bold">1.450</td>
        </tr>
        <tr>
            <td class="text-center font-bold">12</td>
            <td class="font-bold">Motor Mount Bracket for N20 Motor</td>
            <td>قواعد تثبيت هندسية صلبة لتأمين استقامة وصلابة المحركات على الشاسيه</td>
            <td class="text-center font-bold">2</td>
            <td class="price-col">0.650</td>
            <td class="price-col font-bold">1.300</td>
        </tr>
        <tr>
            <td class="text-center font-bold">13</td>
            <td class="font-bold">Battery Holder 2x 18650 Series (ME-4743)</td>
            <td>حامل بطاريتين 18650 موصول توالي لتوليد 7.4V (البطاريات متوفرة لدينا بالجامعة)</td>
            <td class="text-center font-bold">1</td>
            <td class="price-col">1.000</td>
            <td class="price-col font-bold">1.000</td>
        </tr>
        <tr>
            <td class="text-center font-bold">14</td>
            <td class="font-bold">Breakable Male Headers 1x40pin 2.54mm</td>
            <td>رؤوس توصيل قياسية لتسهيل فك وتركيب وتبديل الحساسات لأغراض الصيانة</td>
            <td class="text-center font-bold">4</td>
            <td class="price-col">0.150</td>
            <td class="price-col font-bold">0.600</td>
        </tr>
        <tr class="total-row">
            <td colspan="3" style="text-align: right; font-weight: 800; padding: 5px 8px;">
                المجموع الإجمالي الكلي المعتمد (شامل ضريبة المبيعات 16% والتوصيل المجاني)
            </td>
            <td class="text-center font-bold">25 قطعة</td>
            <td class="text-center font-bold">—</td>
            <td class="price-col total-price font-bold">74.200 د.أ</td>
        </tr>
    </tbody>
</table>

<div class="section-title">ثانياً: المبررات الهندسية والتقنية للاعتماد (Technical Defense)</div>

<div class="justification-box">
    <ul>
        <li><strong>الالتزام التام بقوانين مسابقة MMRC26:</strong> أبعاد الروبوت المجمّع ستكون (حوالي 9 سم × 7.5 سم)، وهو أصغر بكثير من الحد الأقصى المسموح (25 سم × 25 سم)، مما يضمن حركة وانعطافات انسيابية بنصف قطر 3.5 سم داخل خلايا المتاهة (18 سم × 18 سم) دون ملامسة الجدران.</li>
        <li><strong>منظومة التحكم المغلقة بدون إنكودرات (Sensorless Closed-Loop):</strong> تم الاستغناء عن الإنكودرات الميكانيكية المعرضة للكسر وتأخير الشحن، واعتماد جيروسكوب <strong>MPU6050</strong> لتثبيت التوجيه بخط مستقيم (Heading PID)، مدعوماً بحساسات الليزر <strong>VL53L0X</strong> لتوسيط المسار تلقائياً (Wall Centering)، مما يحقق دقة مسار فائقة بتكلفة اقتصادية.</li>
        <li><strong>هندسة منظومة الطاقة المنفصلة (Dual Power Rails):</strong> تغذية المحركات مباشرة بجهد 7.4V عبر درايفر <strong>MX1508</strong> بتقنية MOSFET عالية الكفاءة بدون فقد حراري، مع عزل دائرة تغذية المعالج <strong>ESP32</strong> عبر خافض جهد مستقل (Buck Converter) لمنع إعادة تشغيل المعالج (Reset) أثناء الاندفاع والفرملة.</li>
        <li><strong>الجاهزية البرمجية والخوارزمية:</strong> خوارزميات الاستكشاف (<strong>Modified Flood Fill</strong>) ومسار السرعة الأسرع (<strong>Turn-Weighted A*</strong>) مبرمجة ومختبرة مسبقاً وتعمل بنجاح بنسبة 100% بنظام Zero-Allocation لا يستهلك أكثر من 50 بايت من ذاكرة الروبوت.</li>
    </ul>
</div>

<table class="footer-table">
    <tr>
        <td style="width: 33%; text-align: center;">
            <strong>فريق العمل (الطلبة الباحثين)</strong><br>
            فريق ميكروماوس جامعة جدارا
            <div class="signature-box">التوقيع والتاريخ</div>
        </td>
        <td style="width: 33%; text-align: center;">
            <strong>مشرف المشروع</strong><br>
            الدكتورة المشرفة المحترمة
            <div class="signature-box">الموافقة والاعتماد</div>
        </td>
        <td style="width: 33%; text-align: center;">
            <strong>عمادة كلية الهندسة</strong><br>
            جامعة جدارا
            <div class="signature-box">الختم والتنسيب</div>
        </td>
    </tr>
</table>

</body>
</html>
"""

output_pdf = "docs/MMRC26_Jadara_University_BOM_Quotation.pdf"
root_pdf = "MMRC26_Jadara_University_BOM_Quotation.pdf"

with sync_playwright() as p:
    browser = p.chromium.launch()
    page = browser.new_page()
    page.set_content(html_content, wait_until="networkidle")
    
    pdf_bytes = page.pdf(
        format="A4",
        print_background=True,
        margin={"top": "8mm", "bottom": "8mm", "left": "8mm", "right": "8mm"}
    )
    
    with open(output_pdf, "wb") as f:
        f.write(pdf_bytes)
        
    with open(root_pdf, "wb") as f:
        f.write(pdf_bytes)
        
    browser.close()

print(f"Single-page PDF regenerated successfully: {len(pdf_bytes)} bytes")
