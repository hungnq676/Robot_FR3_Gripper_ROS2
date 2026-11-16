// Builds bao_cao/Bao_cao_NCKH_FR3.docx from the chapter files in bao_cao/noi_dung/ (a small markup):
//   # Heading 1 (new page)   ## Heading 2   ### Heading 3
//   blank-line separated paragraphs, **bold**, *italic*, `code`
//   - bullet        1. numbered (restarts after any other block)
//   ![Hình 3.1. Caption](path/to/img.png|14)      width in cm (default 15)
//   TABLE Bảng 3.1. Caption      then pipe rows (first = header) until a blank line
//   $$ formula $$ (2.1)          centred formula with its number
//   ```  code  ```               monospace block
// Vietnamese thesis layout: A4, margins 2/2/3/2 cm, Times New Roman 13, line spacing 1.5.
const fs = require("fs");
const path = require("path");
const {
  Document, Packer, Paragraph, TextRun, HeadingLevel, AlignmentType, ImageRun, Table, TableRow, TableCell,
  WidthType, BorderStyle, ShadingType, PageBreak, Footer, PageNumber, TableOfContents, LevelFormat,
  PositionalTab, PositionalTabAlignment, PositionalTabRelativeTo, PositionalTabLeader, VerticalAlign,
} = require("docx");

const ROOT = __dirname;
const SRC = path.join(ROOT, "noi_dung");
const FONT = "Times New Roman";
const PT = 13;
const LINE = 360; // 1.5 lines
const CM = 567; // twips per cm
const TEXT_W = 16 * CM; // 21 - 3 - 2 cm

const fig = [], tab = [];

function runs(text, base = {}) {
  // **bold**, *italic*, `code`
  const out = [];
  const re = /(\*\*[^*]+\*\*|\*[^*]+\*|`[^`]+`)/g;
  let last = 0, m;
  while ((m = re.exec(text))) {
    if (m.index > last) out.push(new TextRun({ text: text.slice(last, m.index), font: FONT, size: PT * 2, ...base }));
    const t = m[0];
    if (t.startsWith("**")) out.push(new TextRun({ text: t.slice(2, -2), bold: true, font: FONT, size: PT * 2, ...base }));
    else if (t.startsWith("`")) out.push(new TextRun({ text: t.slice(1, -1), font: "Courier New", size: 22, ...base }));
    else out.push(new TextRun({ text: t.slice(1, -1), italics: true, font: FONT, size: PT * 2, ...base }));
    last = m.index + t.length;
  }
  if (last < text.length) out.push(new TextRun({ text: text.slice(last), font: FONT, size: PT * 2, ...base }));
  return out;
}

const para = (text, opt = {}) => new Paragraph({
  children: runs(text, opt.run || {}),
  alignment: opt.align || AlignmentType.JUSTIFIED,
  spacing: { line: LINE, before: opt.before ?? 0, after: opt.after ?? 120 },
  indent: opt.noIndent ? undefined : { firstLine: Math.round(1 * CM) },
  keepNext: opt.keepNext,
});

let numberedRef = 0;
function blocks(md) {
  const lines = md.replace(/\r/g, "").split("\n");
  const out = [];
  let i = 0, inList = null;
  while (i < lines.length) {
    const l = lines[i];
    if (!l.trim()) { i++; inList = null; continue; }
    if (l.startsWith("# ")) {
      out.push(new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun({ text: l.slice(2).toUpperCase(), font: FONT, size: 30, bold: true })],
        alignment: AlignmentType.CENTER, pageBreakBefore: true, spacing: { before: 0, after: 360, line: LINE } }));
      i++; continue;
    }
    if (l.startsWith("## ")) {
      out.push(new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: l.slice(3), font: FONT, size: 28, bold: true })],
        spacing: { before: 240, after: 120, line: LINE }, keepNext: true }));
      i++; continue;
    }
    if (l.startsWith("### ")) {
      out.push(new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: l.slice(4), font: FONT, size: 26, bold: true, italics: true })],
        spacing: { before: 180, after: 100, line: LINE }, keepNext: true }));
      i++; continue;
    }
    const img = l.match(/^!\[(.+)\]\((.+?)(?:\|([\d.]+))?\)\s*$/);
    if (img) {
      const [_, cap, p, wcm] = img;
      const file = path.isAbsolute(p) ? p : path.join(ROOT, p);
      const buf = fs.readFileSync(file);
      const { w, h } = imgSize(buf);
      const width = (parseFloat(wcm || "15") * CM) / 15; // twips -> px at 96 dpi: 1 px = 15 twips
      const height = (width * h) / w;
      const type = /\.png$/i.test(file) ? "png" : "jpg";
      out.push(new Paragraph({ alignment: AlignmentType.CENTER, keepNext: true, spacing: { before: 120, after: 60 },
        children: [new ImageRun({ type, data: buf, transformation: { width: Math.round(width), height: Math.round(height) } })] }));
      out.push(new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 200, line: 300 },
        children: runs(cap, { italics: true, size: 24 }) }));
      fig.push(cap);
      i++; continue;
    }
    if (l.startsWith("TABLE ")) {
      const cap = l.slice(6).trim();
      tab.push(cap);
      out.push(new Paragraph({ alignment: AlignmentType.CENTER, keepNext: true, spacing: { before: 160, after: 80, line: 300 },
        children: runs(cap, { italics: true, size: 24 }) }));
      i++;
      const rows = [];
      while (i < lines.length && lines[i].trim().startsWith("|")) {
        const cells = lines[i].trim().replace(/^\||\|$/g, "").split("|").map(c => c.trim());
        if (!cells.every(c => /^:?-+:?$/.test(c))) rows.push(cells);
        i++;
      }
      out.push(table(rows));
      out.push(new Paragraph({ spacing: { after: 120 }, children: [] }));
      continue;
    }
    if (l.startsWith("$$")) {
      const m = l.match(/^\$\$(.+)\$\$\s*(\(.+\))?\s*$/);
      out.push(new Paragraph({ alignment: AlignmentType.CENTER, spacing: { before: 120, after: 120, line: LINE },
        children: [...runs(m[1].trim(), { italics: true }), new TextRun({ text: m[2] ? "        " + m[2] : "", font: FONT, size: PT * 2 })] }));
      i++; continue;
    }
    if (l.startsWith("```")) {
      i++;
      const code = [];
      while (i < lines.length && !lines[i].startsWith("```")) code.push(lines[i++]);
      i++;
      code.forEach((c, k) => out.push(new Paragraph({ spacing: { line: 240, before: k ? 0 : 80, after: k === code.length - 1 ? 160 : 0 },
        shading: { type: ShadingType.CLEAR, fill: "F3F3F3", color: "auto" }, indent: { left: 200 },
        children: [new TextRun({ text: c || " ", font: "Courier New", size: 18 })] })));
      continue;
    }
    if (/^- /.test(l)) {
      out.push(new Paragraph({ numbering: { reference: "bullet", level: 0 }, spacing: { line: LINE, after: 60 },
        alignment: AlignmentType.JUSTIFIED, children: runs(l.slice(2)) }));
      i++; continue;
    }
    if (/^ {2,}- /.test(l)) {
      out.push(new Paragraph({ numbering: { reference: "bullet", level: 1 }, spacing: { line: LINE, after: 40 },
        alignment: AlignmentType.JUSTIFIED, children: runs(l.trim().slice(2)) }));
      i++; continue;
    }
    if (/^\d+\. /.test(l)) {
      if (inList !== "num") { numberedRef++; inList = "num"; }
      out.push(new Paragraph({ numbering: { reference: `num${numberedRef}`, level: 0 }, spacing: { line: LINE, after: 60 },
        alignment: AlignmentType.JUSTIFIED, children: runs(l.replace(/^\d+\. /, "")) }));
      i++; continue;
    }
    // paragraph: join until blank line
    const buf = [l];
    i++;
    while (i < lines.length && lines[i].trim() && !/^(#|!\[|TABLE |\$\$|```|- |\d+\. | {2,}- )/.test(lines[i])) buf.push(lines[i++]);
    out.push(para(buf.join(" ")));
    inList = null;
  }
  return out;
}

function table(rows) {
  const n = rows[0].length;
  // widths proportional to the longest text in each column, min 1.6 cm
  const lens = Array.from({ length: n }, (_, c) => Math.max(...rows.map(r => (r[c] || "").length)));
  const total = lens.reduce((a, b) => a + Math.max(b, 6), 0);
  // never narrower than the longest single word, so names like MOVE_TIME_WRITE are not split
  const word = Array.from({ length: n }, (_, c) => Math.max(...rows.map(r => Math.max(0, ...(r[c] || "").replace(/\*\*|`/g, "").split(/\s+/).map(t => t.length)))));
  const minW = word.map(x => Math.max(Math.round((x * 0.25 + 0.5) * CM), Math.round(1.6 * CM)));
  let widths = lens.map((x, c) => Math.max(Math.round((Math.max(x, 6) / total) * TEXT_W), minW[c]));
  const s = widths.reduce((a, b) => a + b, 0);
  if (s > TEXT_W) {
    // shrink only the columns wider than their minimum
    const over = s - TEXT_W, slack = widths.reduce((a, w, c) => a + w - minW[c], 0);
    widths = widths.map((w, c) => Math.round(w - (slack > 0 ? (over * (w - minW[c])) / slack : (over * w) / s)));
  }
  widths[n - 1] += TEXT_W - widths.reduce((a, b) => a + b, 0);
  const border = { style: BorderStyle.SINGLE, size: 4, color: "888888" };
  const borders = { top: border, bottom: border, left: border, right: border };
  return new Table({
    width: { size: TEXT_W, type: WidthType.DXA }, columnWidths: widths,
    rows: rows.map((r, ri) => new TableRow({
      tableHeader: ri === 0, cantSplit: true,
      children: widths.map((w, c) => new TableCell({
        width: { size: w, type: WidthType.DXA }, borders, verticalAlign: VerticalAlign.CENTER,
        shading: ri === 0 ? { type: ShadingType.CLEAR, fill: "DCE6F1", color: "auto" } : undefined,
        margins: { top: 60, bottom: 60, left: 100, right: 100 },
        children: [new Paragraph({ spacing: { line: 276, after: 0 }, alignment: ri === 0 ? AlignmentType.CENTER : AlignmentType.LEFT,
          children: runs(r[c] || "", { size: 22, bold: ri === 0 ? true : undefined }) })],
      })),
    })),
  });
}

function imgSize(buf) {
  if (buf[0] === 0x89) return { w: buf.readUInt32BE(16), h: buf.readUInt32BE(20) };
  let o = 2; // JPEG: find SOF marker
  while (o < buf.length) {
    const marker = buf[o + 1], len = buf.readUInt16BE(o + 2);
    if (marker >= 0xc0 && marker <= 0xc3) return { h: buf.readUInt16BE(o + 5), w: buf.readUInt16BE(o + 7) };
    o += 2 + len;
  }
  throw new Error("image size?");
}

const center = (text, size, opt = {}) => new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: opt.after ?? 120, before: opt.before ?? 0, line: 300 },
  children: [new TextRun({ text, font: FONT, size, bold: opt.bold, italics: opt.italics })] });

function cover() {
  // Form copied from the faculty template (27-SV-2026-DT1): double page border, logo, centred title block,
  // advisor/students with a hanging indent, "HÀ NỘI – 2026" at the bottom. Content comes from thong_tin.json.
  const m = JSON.parse(fs.readFileSync(path.join(SRC, "thong_tin.json"), "utf8"));
  const c = (text, size, before, after, extra = {}) => new Paragraph({ alignment: AlignmentType.CENTER, spacing: { before, after, line: 276 }, ...extra,
    children: text === null ? [] : [new TextRun({ text, font: FONT, size, bold: true })] });
  const logo = fs.readFileSync(path.join(ROOT, "hinh", "logo_ptit.png"));
  const person = (label, name) => new Paragraph({ spacing: { after: 80, line: 276 }, indent: { left: 3515, hanging: 2948 },
    tabStops: [{ type: "left", position: 3515 }],
    children: [new TextRun({ text: label, font: FONT, size: 24, bold: true }), new TextRun({ text: "\t" + name, font: FONT, size: 24, bold: true })] });
  const gv = m.nguoi.find(([a]) => a.startsWith("Người hướng dẫn"));
  const sv = m.nguoi.filter(([a]) => !a.startsWith("Người hướng dẫn")).map(([a, b]) => a.startsWith("Chủ trì") ? b + " (chủ trì)" : b);
  return [
    c(m.co_quan, 28, 0, 0),
    c(m.truong, 28, 160, 40),
    new Paragraph({ spacing: { before: 120, after: 120 }, indent: { left: 3118, right: 3118 },
      border: { bottom: { style: BorderStyle.SINGLE, size: 12, space: 1, color: "1F3864" } }, children: [] }),
    new Paragraph({ alignment: AlignmentType.CENTER, spacing: { before: 320 },
      children: [new ImageRun({ type: "png", data: logo, transformation: { width: 129, height: 131 } })] }),
    c("BÁO CÁO NGHIÊN CỨU KHOA HỌC SINH VIÊN", 28, 520, 280),
    c("ĐỀ TÀI:", 28, 0, 200),
    ...m.ten_de_tai.map((t, k) => c(t, 28, 0, k === m.ten_de_tai.length - 1 ? 320 : 80)),
    c((m.ma_so || "").replace(/\s*–\s*/g, "-"), 24, 0, 680),
    person("Giảng viên hướng dẫn:", gv ? gv[1] : ""),
    ...sv.map((b, k) => person(k === 0 ? "Sinh viên thực hiện:" : "", b)),
    c("HÀ NỘI – " + ((m.noi_ngay || "").match(/\d{4}/) || ["2026"])[0], 28, 1200, 0),
  ];
}

function frontPage(title, file) {
  return [new Paragraph({ heading: HeadingLevel.HEADING_1, pageBreakBefore: true, alignment: AlignmentType.CENTER, spacing: { after: 360 },
    children: [new TextRun({ text: title, font: FONT, size: 30, bold: true })] }), ...blocks(fs.readFileSync(path.join(SRC, file), "utf8"))];
}

function listPage(title, items) {
  return [new Paragraph({ heading: HeadingLevel.HEADING_1, pageBreakBefore: true, alignment: AlignmentType.CENTER, spacing: { after: 360 },
    children: [new TextRun({ text: title, font: FONT, size: 30, bold: true })] }),
    ...items.map(t => new Paragraph({ spacing: { line: 300, after: 80 }, indent: { left: 400, hanging: 400 }, children: runs(t, { size: 24 }) }))];
}

const chapters = fs.readdirSync(SRC).filter(f => /^chuong_\d.*\.md$/.test(f)).sort();
const body = [];
for (const f of chapters) body.push(...blocks(fs.readFileSync(path.join(SRC, f), "utf8")));
const tail = [];
for (const f of ["tai_lieu_tham_khao.md", "phu_luc.md"]) tail.push(...blocks(fs.readFileSync(path.join(SRC, f), "utf8")));

const numbering = [{ reference: "bullet", levels: [
  { level: 0, format: LevelFormat.BULLET, text: "–", alignment: AlignmentType.LEFT, style: { paragraph: { indent: { left: 720, hanging: 360 } } } },
  { level: 1, format: LevelFormat.BULLET, text: "+", alignment: AlignmentType.LEFT, style: { paragraph: { indent: { left: 1080, hanging: 360 } } } }] }];
for (let k = 1; k <= numberedRef; k++) numbering.push({ reference: `num${k}`, levels: [
  { level: 0, format: LevelFormat.DECIMAL, text: "%1.", alignment: AlignmentType.LEFT, style: { paragraph: { indent: { left: 720, hanging: 360 } } } }] });

const page = { size: { width: 11906, height: 16838 }, margin: { top: 2 * CM, bottom: 2 * CM, left: 3 * CM, right: 2 * CM } };
const footer = new Footer({ children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ children: [PageNumber.CURRENT], font: FONT, size: 24 })] })] });

const doc = new Document({
  creator: "", title: "Báo cáo NCKH: hệ thống gắp vật theo lệnh ngôn ngữ với tay máy FR3",
  features: { updateFields: true },
  styles: {
    default: { document: { run: { font: FONT, size: PT * 2 } } },
    paragraphStyles: [
      { id: "Heading1", name: "Heading 1", basedOn: "Normal", next: "Normal", quickFormat: true, run: { font: FONT, size: 30, bold: true }, paragraph: { outlineLevel: 0 } },
      { id: "Heading2", name: "Heading 2", basedOn: "Normal", next: "Normal", quickFormat: true, run: { font: FONT, size: 28, bold: true }, paragraph: { outlineLevel: 1 } },
      { id: "Heading3", name: "Heading 3", basedOn: "Normal", next: "Normal", quickFormat: true, run: { font: FONT, size: 26, bold: true, italics: true }, paragraph: { outlineLevel: 2 } },
    ],
  },
  numbering: { config: numbering },
  sections: [
    { properties: { page: { size: page.size, margin: { top: 1417, right: 1134, bottom: 1417, left: 1984 },
        borders: Object.fromEntries(["pageBorderTop", "pageBorderLeft", "pageBorderBottom", "pageBorderRight"].map((k) =>
          [k, { style: BorderStyle.THICK_THIN_SMALL_GAP, size: 24, space: 18, color: "000000" }])) } }, children: cover() },
    { properties: { page }, footers: { default: footer }, children: [
      ...frontPage("LỜI CAM ĐOAN", "cam_doan.md"),
      ...frontPage("LỜI CẢM ƠN", "cam_on.md"),
      new Paragraph({ pageBreakBefore: true, alignment: AlignmentType.CENTER, spacing: { after: 240 }, children: [new TextRun({ text: "MỤC LỤC", font: FONT, size: 30, bold: true })] }),
      new TableOfContents("MỤC LỤC", { hyperlink: true, headingStyleRange: "1-3" }),
      ...listPage("DANH MỤC HÌNH VẼ", fig),
      ...listPage("DANH MỤC BẢNG", tab),
      ...frontPage("DANH MỤC TỪ VIẾT TẮT", "viet_tat.md"),
      ...body, ...tail,
    ] },
  ],
});

Packer.toBuffer(doc).then(b => {
  const out = path.join(ROOT, "Bao_cao_NCKH_FR3.docx");
  fs.writeFileSync(out, b);
  console.log(out, `${fig.length} hình, ${tab.length} bảng`);
});
