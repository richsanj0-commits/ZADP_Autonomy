# Builds docs/ZADP_Theorems_and_References.pdf : every method/theorem ZADP uses, stage-wise, with weightage and the paper to cite.
from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak
from reportlab.lib.units import mm

ss = getSampleStyleSheet()
H1, H2, B = ss["Title"], ss["Heading2"], ss["BodyText"]
C = ParagraphStyle("c", parent=B, fontSize=8.2, leading=10)
CB = ParagraphStyle("cb", parent=C, fontName="Helvetica-Bold")
P = lambda t, s=C: Paragraph(t, s)

STAGES = [
 ("Stage 1 - Sketch to real shape (Round 1)", [
  ("Otsu thresholding (ink vs paper)", "*", "N. Otsu, 'A threshold selection method from gray-level histograms', IEEE Trans. Systems, Man, and Cybernetics 9(1), 1979, pp. 62-66."),
  ("Flood fill / connected components", "*", "A. Rosenfeld, J. Pfaltz, 'Sequential operations in digital picture processing', J. ACM 13(4), 1966."),
  ("Morphological closing", "*", "J. Serra, Image Analysis and Mathematical Morphology, Academic Press, 1982."),
  ("Moore-neighbour boundary tracing", "*", "R. C. Gonzalez, R. E. Woods, Digital Image Processing (textbook, boundary following)."),
  ("Corner detection by turning angle; line fit (least squares)", "**", "C.-H. Teh, R. T. Chin, 'On the detection of dominant points on digital curves', IEEE TPAMI 11(8), 1989."),
  ("Kasa algebraic circle fit (arcs)", "**", "I. Kasa, 'A circle fitting procedure and its error analysis', IEEE Trans. Instrum. Meas. IM-25(1), 1976, pp. 8-14."),
  ("PCA for text-label orientation", "*", "I. T. Jolliffe, Principal Component Analysis, 2nd ed., Springer, 2002."),
  ("Tesseract OCR (LSTM)", "*", "R. Smith, 'An Overview of the Tesseract OCR Engine', Proc. ICDAR 2007, pp. 629-633."),
  ("Minimum-norm least-squares polygon closure x = A^T (A A^T)^-1 b (Moore-Penrose)", "**", "R. Penrose, 'A generalized inverse for matrices', Proc. Cambridge Phil. Soc. 51, 1955; G. Golub, C. Van Loan, Matrix Computations."),
 ]),
 ("Stage 2 - Pile size and truck margins (Round 2)", [
  ("Cone volume V = (1/3) pi r^2 h, h = r tan(theta) => r = cbrt(3V / (pi tan theta))", "**", "Elementary geometry; angle of repose: R. M. Nedderman, Statics and Kinematics of Granular Materials, CUP, 1992."),
  ("Loose density / swell of blasted rock", "*", "Caterpillar Performance Handbook (industry reference); or W. Hustrulid, M. Kuchta, Open Pit Mine Planning and Design."),
 ]),
 ("Stage 3 - Safe zone and partition (Round 3)  [ZADP Steps 1-2]", [
  ("Minkowski sum / erosion (configuration space of the truck)", "***", "T. Lozano-Perez, 'Spatial planning: a configuration space approach', IEEE Trans. Computers C-32(2), 1983."),
  ("Polygon offsetting and boolean operations (Vatti clipping; Clipper library)", "**", "B. R. Vatti, 'A generic solution to polygon clipping', Comm. ACM 35(7), 1992; A. Johnson, Clipper library."),
  ("Boustrophedon cellular decomposition (critical points of a sweep line)", "***", "H. Choset, P. Pignon, 'Coverage path planning: the boustrophedon cellular decomposition', Field and Service Robotics, 1998; H. Choset, 'Coverage of known spaces: the boustrophedon cellular decomposition', Autonomous Robots 9, 2000."),
  ("Coverage path planning survey (context)", "*", "E. Galceran, M. Carreras, 'A survey on coverage path planning for robotics', Robotics and Autonomous Systems 61(12), 2013."),
 ]),
 ("Stage 4 - Roads and pile spots (Round 4)  [ZADP Step 3]", [
  ("Hexagonal packing is the densest packing of equal circles: density pi/sqrt(12) = 0.9069 (Thue's theorem)", "***", "A. Thue, 1910; first complete proof L. Fejes Toth, 'Uber die dichteste Kugellagerung', Math. Z. 48, 1942/43; also Hai-Chau Chang, Lih-Chung Wang, 'A simple proof of Thue's theorem on circle packing', arXiv:1009.4322, 2010."),
  ("Herringbone (spine + branch) road layout", "*", "Design choice; compare warehouse 'fishbone' aisles: K. R. Gue, R. D. Meller, 'Aisle configurations for unit-load warehouses', IIE Transactions 41, 2009."),
 ]),
 ("Stage 5 - Dispatch and safety (Round 5)  [ZADP Steps 4-5]", [
  ("A* shortest path search", "**", "P. E. Hart, N. J. Nilsson, B. Raphael, 'A formal basis for the heuristic determination of minimum cost paths', IEEE Trans. SSC 4(2), 1968."),
  ("Breadth-first search reachability", "*", "E. F. Moore, 'The shortest path through a maze', 1959; Cormen et al., Introduction to Algorithms."),
  ("Greedy tangency (touching) placement of circles", "***", "W. Q. Huang, Y. Li, et al., 'Greedy algorithms for packing unequal circles into a rectangular container', J. Operational Research Society 56, 2005; Hifi & M'Hallah, 'A literature review on circle and sphere packing problems', Advances in Operations Research, 2009."),
  ("Mutual exclusion (lane mutex) and deadlock: Coffman conditions; wait-for-graph cycle detection", "***", "E. G. Coffman, M. Elphick, A. Shoshani, 'System deadlocks', ACM Computing Surveys 3(2), 1971; E. W. Dijkstra, 'Cooperating sequential processes', 1965."),
  ("Zone locking / reserved routes for multi-vehicle traffic (AGV-style zone control)", "**", "I. F. A. Vis, 'Survey of research in the design and control of automated guided vehicle systems', EJOR 170, 2006."),
  ("Mine haul-truck dispatching (context)", "*", "M. Alarie, M. Gamache, 'Overview of solution strategies used in truck dispatching systems for open pit mines', Int. J. Surface Mining 16(1), 2002."),
  ("Seeded pseudo-random generator (repeatable runs)", "*", "Mulberry32 style generator; P. L'Ecuyer, 'Random number generation', Handbook of Computational Statistics, 2012."),
 ]),
 ("Stage 6 - Evaluation and defence of the result", [
  ("Upper bound: coverage <= 0.9069 x coverable area (Thue / Fejes Toth)", "***", "As Stage 4."),
  ("Binary circle packings cannot beat hexagonal density when radius ratio >= 0.742", "**", "G. Blind, 'Uber Unterdeckungen der Ebene durch Kreise', J. Reine Angew. Math. 236, 1969 (verify exact statement)."),
  ("Random uncoordinated placement saturates at ~54.7 % (random sequential adsorption)", "**", "J. Feder, 'Random sequential adsorption', J. Theor. Biol. 87, 1980; G. Zhang, S. Torquato, 'Precise algorithm to generate random sequential addition of hard hyperspheres at saturation', Phys. Rev. E 88, 2013."),
  ("Optimal packing in a region is NP-hard (why bounds are used instead of proving optimality)", "***", "R. J. Fowler, M. S. Paterson, S. L. Tanimoto, 'Optimal packing and covering in the plane are NP-complete', Information Processing Letters 12(3), 1981."),
  ("Saturated (maximal) packing / jamming as local-optimality evidence", "*", "S. Torquato, F. H. Stillinger, 'Jammed hard-particle packings', Rev. Mod. Phys. 82, 2010."),
  ("Mean and 95 % confidence interval over random seeds (Student t)", "*", "Any statistics text, e.g. D. C. Montgomery, Design and Analysis of Experiments."),
 ]),
]

W8 = [("***", "Core claim of the research - must be cited"), ("**", "Important method - should be cited"), ("*", "Supporting tool - textbook citation is enough")]

doc = SimpleDocTemplate("docs/ZADP_Theorems_and_References.pdf", pagesize=A4, leftMargin=14*mm, rightMargin=14*mm, topMargin=14*mm, bottomMargin=14*mm, title="ZADP Theorems and References")
els = [Paragraph("ZADP - Theorems, Algorithms and References", H1),
       P("Every method and theorem used by the ZADP dump-planning simulator, stage by stage, with its weightage and the paper to cite. "
         "References are given from the authors' knowledge and must be checked against the published source before submission.", B),
       Spacer(1, 4*mm)]
t = Table([[P("<b>Weight</b>"), P("<b>Meaning</b>")]] + [[P(a), P(b)] for a, b in W8], colWidths=[20*mm, 150*mm])
t.setStyle(TableStyle([("GRID", (0,0), (-1,-1), 0.4, colors.grey), ("BACKGROUND", (0,0), (-1,0), colors.HexColor("#dde6f0"))]))
els += [t, Spacer(1, 5*mm)]
n = 0
for title, rows in STAGES:
    els.append(Paragraph(title, H2))
    data = [[P("<b>#</b>"), P("<b>Method / theorem</b>"), P("<b>Weight</b>"), P("<b>Paper to cite</b>")]]
    for m, w, ref in rows:
        n += 1; data.append([P(str(n)), P(m, CB if w == "***" else C), P(w), P(ref)])
    t = Table(data, colWidths=[8*mm, 62*mm, 14*mm, 98*mm], repeatRows=1)
    t.setStyle(TableStyle([("GRID", (0,0), (-1,-1), 0.4, colors.grey), ("BACKGROUND", (0,0), (-1,0), colors.HexColor("#dde6f0")), ("VALIGN", (0,0), (-1,-1), "TOP")]))
    els += [t, Spacer(1, 3*mm)]
els += [Paragraph("Core proof chain for the coverage claim", H2),
        P("1. Trucks are points in configuration space after Minkowski erosion (Lozano-Perez) -> pile centres must lie in the safe zone.<br/>"
          "2. Coverable ground C = safe zone grown by the pile radius, cut to the area.<br/>"
          "3. Thue / Fejes Toth: touching equal circles cover at most 90.69 % of any region -> coverage <= 0.9069 x C.<br/>"
          "4. Optimal packing is NP-hard (Fowler et al.) -> global optimality cannot be proved; the standard evidence is the bound plus a saturated layout.<br/>"
          "5. ZADP reaches ~76 % vs ~88 % cap (~85 % of the cap), leaves no reachable free spot, and beats random placement (54.7 %, Feder) and fixed spots (~17 %).<br/>"
          "6. Safety: lane mutex + zone locking break the Coffman deadlock conditions; wait-for cycles are detected; collisions are measured (0 in all runs).", B)]
doc.build(els)
print("ok", n, "items")
