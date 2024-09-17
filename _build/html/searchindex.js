Search.setIndex({"docnames": ["intro", "lec1/intro", "lec10/diff-trans", "lec11-12/jacobian", "lec13/singularity", "lec14/statics", "lec15/inverse_vk", "lec16-17/nik", "lec18/dyn", "lec19/control_overview", "lec2/configuration", "lec20/djc", "lec21/cjc", "lec22/osc", "lec3-5/basic-kinematics", "lec6-8/fk", "lec9/ik", "markdown", "markdown-notebooks", "notebooks"], "filenames": ["intro.md", "lec1/intro.md", "lec10/diff-trans.md", "lec11-12/jacobian.md", "lec13/singularity.md", "lec14/statics.md", "lec15/inverse_vk.md", "lec16-17/nik.md", "lec18/dyn.md", "lec19/control_overview.md", "lec2/configuration.md", "lec20/djc.md", "lec21/cjc.md", "lec22/osc.md", "lec3-5/basic-kinematics.md", "lec6-8/fk.md", "lec9/ik.md", "markdown.md", "markdown-notebooks.md", "notebooks.ipynb"], "titles": ["Welcome", "Introduction", "Derivative of Transformation", "Jacobian", "Singularity &amp; Redundancy", "Statics", "Inverse Velocity Kinematics", "Numerical Inverse Kinematics", "Dynamics (Lagrange formulation)", "Overview of Robot Control", "Robot Configuration", "Decentralized Joint Control", "Centralized Joint Control", "Operational Space Control", "Basic Kinematics", "Forward Kinematics", "Inverse Kinematics", "Markdown Files", "Notebooks with MyST Markdown", "Content with notebooks"], "terms": {"model": [0, 8, 11, 12], "control": [0, 15, 16], "robot": [0, 6, 8, 14, 16], "mae": 0, "547": 0, "fall": 0, "2024": 0, "cover": 0, "theori": [0, 11], "method": [0, 12, 13], "homogen": [0, 3, 15], "transform": [0, 3, 9, 11, 13, 15, 16], "direct": [0, 3, 4, 6, 7, 12, 13, 15, 18], "invers": [0, 9, 14], "kinemat": [0, 3, 4, 5, 8, 9, 10, 13], "jacobian": [0, 4, 5, 6, 8, 13], "static": [0, 9], "dynam": [0, 7, 11], "arm": [0, 3, 6], "discuss": [0, 12, 13], "plan": 0, "bruno": 0, "siciliano": 0, "lorenzo": 0, "sciavicco": 0, "luigi": 0, "villani": 0, "giusepp": 0, "oriolo": 0, "springer": 0, "2009": 0, "modern": 0, "mechan": [0, 2, 5, 6, 9, 10, 15], "kevin": 0, "m": [0, 6, 9, 10, 11, 15], "lynch": 0, "frank": 0, "c": [0, 3, 4, 5, 7, 9, 10, 11, 12, 13, 14, 15, 16], "park": [0, 14], "cambridg": 0, "univers": [0, 10], "press": 0, "2017": 0, "avail": [0, 15], "an": [0, 3, 4, 6, 7, 8, 10, 12, 13, 14, 15, 16, 17], "open": [0, 8, 9, 10, 11], "sourc": 0, "resourc": 0, "The": [0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18], "major": 0, "its": [0, 4, 5, 6, 7, 8, 10, 11, 12, 14, 15, 16], "content": [0, 17, 18], "includ": [0, 5, 8, 10, 15, 18, 19], "figur": [0, 2, 3, 10, 11, 16], "ar": [0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18], "directli": [0, 6, 9, 11, 12, 13], "adapt": 0, "from": [0, 2, 3, 4, 6, 8, 10, 11, 12, 13, 14, 15, 16, 19], "et": 0, "al": 0, "s": [0, 2, 3, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18, 19], "book": [0, 15, 17, 18, 19], "few": [0, 10], "purpos": [0, 7, 16, 17], "provid": [0, 6, 8, 10, 11, 15], "concis": 0, "materi": 0, "specif": [0, 4, 7, 10, 16, 17], "tailor": 0, "student": 0, "simultan": 0, "aim": 0, "benefit": [0, 9], "gener": [0, 2, 4, 5, 7, 8, 12, 13, 16], "interest": 0, "offer": 0, "quick": 0, "access": [0, 10], "learn": [0, 15, 16], "refer": [0, 2, 3, 5, 8, 9, 11, 12, 13, 14, 15, 16, 17], "review": 0, "elimin": 0, "need": [0, 3, 4, 6, 7, 10, 14, 15, 18], "read": 0, "entir": [0, 4, 5], "nevertheless": [0, 7], "i": [0, 2, 3, 4, 5, 6, 7, 9, 10, 11, 14, 15, 16], "highli": 0, "encourag": 0, "everyon": 0, "explor": 0, "origin": [0, 2, 8, 14, 15, 16], "when": [0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 15, 16, 17, 18], "time": [0, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15], "permit": [0, 16], "am": 0, "assist": 0, "professor": 0, "ira": 0, "A": [0, 3, 6, 7, 8, 10, 11, 13, 14, 15, 16], "fulton": 0, "school": 0, "engin": 0, "arizona": 0, "state": [0, 5, 9, 11, 12, 13, 19], "pi": [0, 4, 6, 15, 16], "intellig": 0, "interact": [0, 19], "system": [0, 6, 8, 11, 12, 13, 14], "webpag": 0, "http": [0, 15], "asu": 0, "github": [0, 15], "io": [0, 15], "focu": 0, "human": [0, 15, 17], "align": [0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 14, 15, 16, 19], "we": [0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16], "develop": 0, "empow": 0, "abil": [0, 15], "effici": [0, 9], "understand": [0, 18], "understood": [0, 14], "user": 0, "through": [0, 5, 7, 8, 9, 13], "varieti": 0, "physic": [0, 2, 3, 6, 8], "how": [0, 3, 6, 8, 12, 13, 15, 18], "can": [0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19], "aptli": 0, "respond": 0, "collabor": 0, "meaningfulli": 0, "contact": [0, 8, 11], "rich": 0, "manipul": [0, 3, 4, 5, 7, 8, 10, 11, 12, 13, 15], "leverag": 0, "principl": [0, 5, 9, 15], "deriv": [0, 3, 6, 7, 8, 11, 12, 13, 14, 15, 16], "represent": [0, 3, 5, 7], "enabl": 0, "surround": 0, "algorithm": [0, 9, 12], "robustli": 0, "fundament": [0, 4, 16], "achiev": [0, 4, 6, 7, 14, 15, 16], "safe": 0, "robust": [0, 12], "our": [0, 6, 14], "lie": [0, 10], "intersect": [0, 10, 15, 16], "base": [0, 2, 3, 6, 8, 9, 11, 12, 13, 15, 16, 18], "optim": [0, 6, 7], "data": [0, 15, 19], "driven": [0, 11], "approach": [0, 12, 15], "har": 0, "complementari": 0, "both": [0, 2, 6, 7, 8, 9, 11, 13, 14, 15, 17], "activ": 0, "look": [0, 12, 13, 14], "enthusiast": 0, "join": 0, "team": 0, "If": [0, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 18], "you": [0, 9, 17, 18, 19], "re": 0, "becom": [0, 12], "part": [0, 4, 7, 8, 11, 14], "pleas": [0, 12, 13, 15], "submit": [0, 7], "your": [0, 17, 18, 19], "applic": 0, "here": [0, 4, 7, 8, 9, 11, 12, 14, 17, 19], "present": [1, 5, 16], "consid": [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16], "vari": [2, 3, 5, 15], "rotat": [2, 3, 4, 5, 6, 7, 8, 15], "matrix": [2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 15], "boldsymbol": [2, 3, 4, 5, 6, 7, 9, 11, 12, 13, 14, 15, 16], "r": [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16], "t": [2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 19], "One": [2, 7, 8, 12, 14, 15], "ha": [2, 3, 4, 6, 8, 9, 10, 11, 13, 14, 15, 16], "differenti": [2, 3, 5, 7, 8, 12, 13, 16], "abov": [2, 3, 4, 5, 6, 8, 9, 11, 12, 13, 14, 15, 16], "equat": [2, 3, 6, 7, 9, 11, 12, 13, 14, 16], "respect": [2, 3, 4, 5, 7, 8, 10, 12, 13, 14, 15, 16], "give": [2, 3, 5, 7, 8, 12, 14, 15, 16], "dot": [2, 3, 4, 5, 6, 7, 9, 11, 12, 13], "o": [2, 3, 5, 6, 7, 8, 14, 16], "newli": 2, "defin": [2, 3, 4, 7, 8, 12, 13, 14, 15, 18], "being": [2, 7, 14, 17], "call": [2, 3, 6, 7, 10, 14, 15, 17], "3": [2, 3, 4, 6, 7, 10, 11, 12, 13, 14, 15, 16], "skew": [2, 7, 12, 13], "symmetr": [2, 7, 8, 12, 13], "next": [2, 3, 7, 14], "let": [2, 4, 5, 7, 8, 9, 10, 11, 14, 18], "find": [2, 3, 6, 7, 10, 11, 12, 13, 14, 15, 16], "interpret": [2, 3, 7, 8, 11, 14], "repres": [2, 3, 4, 6, 7, 8, 10, 11, 14], "move": [2, 5, 6, 8, 10, 11, 15], "frame": [2, 3, 5, 8, 15, 16], "x": [2, 3, 4, 7, 8, 9, 10, 13, 14, 15, 16], "y": [2, 4, 8, 10, 12, 13, 14, 15, 16], "z": [2, 3, 4, 8, 14, 15, 16], "fix": [2, 8, 10, 11, 15, 19], "xyz": [2, 14], "It": [2, 5, 6, 7, 8, 10, 11, 14, 15, 17], "mean": [2, 3, 4, 6, 7, 8, 10, 11, 13, 19], "follow": [2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18], "coordin": [2, 5, 8, 10, 14, 15], "p": [2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 16], "prime": [2, 14, 15], "take": [2, 4, 8, 12, 13, 14, 15], "side": [2, 8, 12, 13, 15], "yield": [2, 8, 12, 16], "where": [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16], "have": [2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 14, 15, 16, 18], "us": [2, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18], "recal": [2, 6, 7, 8, 10, 12, 13], "cours": 2, "given": [2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 16], "omega": [2, 3, 4, 5, 7, 8], "left": [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16], "begin": [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 19], "arrai": [2, 3, 4, 5, 7, 8, 14, 15, 16, 19], "lll": [2, 3, 4, 7, 8, 14, 16], "omega_": [2, 3, 7, 11, 12], "end": [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 19], "right": [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16], "also": [2, 6, 7, 8, 10, 11, 13, 14, 15, 16, 17, 18, 19], "write": [2, 3, 11, 14, 15, 17, 18], "compar": [2, 8, 15], "two": [2, 4, 5, 6, 9, 10, 11, 13, 14, 15, 16, 17, 18], "conclud": [2, 13, 14], "ccc": [2, 3, 4, 7, 8, 14], "0": [2, 3, 4, 5, 6, 7, 8, 11, 12, 13, 14, 15, 16, 19], "henc": [2, 5, 8, 11, 14, 16], "note": [2, 4, 7, 11, 12, 13, 14, 17], "express": [2, 3, 7, 8, 10, 12, 13, 14, 15, 16], "properti": [2, 9, 12, 13, 14], "denot": [2, 7, 8, 9, 11, 14, 15], "shown": [2, 4, 6, 7, 9, 11, 12, 14, 15, 16], "relat": [2, 3, 4, 5, 9, 11, 16], "hold": [2, 4, 8, 9, 10, 11, 14, 16], "map": [2, 3, 4, 5, 14], "point": [2, 4, 6, 7, 8, 10, 14, 15, 16], "o_1": [2, 14], "x_1y_1z_1": 2, "o_0": 2, "x_0y_0z_0": 2, "_": [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16], "1": [2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16, 19], "sinc": [2, 3, 4, 6, 7, 8, 9, 10, 12, 13, 14, 15, 16], "notic": [2, 5, 8, 12, 14, 16], "accord": [2, 5, 7, 9, 14, 15, 16], "dh": 2, "convent": [2, 8, 14], "between": [2, 3, 4, 5, 7, 8, 10, 12, 13, 15], "below": [2, 7, 8, 10, 11, 12, 13, 14, 15, 16], "character": [2, 5, 9], "posit": [2, 3, 6, 7, 8, 10, 12, 13, 14, 15, 16], "v": [2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13], "which": [2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 18], "function": [2, 3, 4, 6, 7, 8, 11, 12, 13, 15, 16, 17], "translat": [2, 5, 8, 15], "Its": [2, 7, 10, 16], "second": [2, 6, 12, 13, 14, 15], "term": [2, 3, 4, 5, 6, 7, 8, 11, 12, 14, 15, 16], "hand": [2, 3, 5, 10, 12, 14, 15], "rewritten": [2, 11], "Then": [2, 3, 5, 7, 9, 10, 12, 13, 14, 15, 16], "lead": [2, 5, 7, 8, 11, 12, 13, 15, 16], "differ": [2, 5, 7, 14, 15, 16, 17], "type": [2, 7, 8, 9, 11, 12, 13, 14, 15, 16], "joint": [2, 3, 4, 5, 6, 7, 8, 13, 15, 16], "conclus": 2, "prismat": [2, 3, 8, 10, 15], "d": [2, 3, 5, 7, 8, 9, 10, 11, 12, 13, 18], "revolut": [2, 3, 4, 8, 10, 15, 16], "vartheta": [2, 3, 7, 8, 14, 15], "n": [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 19], "dof": [3, 4, 8, 15, 16], "e": [3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16], "q": [3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16], "cc": [3, 5, 7, 8, 14], "mathbf": [3, 7, 8, 12, 13, 14, 16], "q_": [3, 6, 8, 15, 16], "ldot": [3, 8, 10, 12, 14, 15], "goal": [3, 11, 14], "relationship": [3, 5, 7, 8, 9, 11, 12, 13, 14, 15], "veloc": [3, 4, 7, 8, 9, 12, 13], "effector": [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16], "In": [3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17], "other": [3, 5, 6, 8, 9, 10, 15, 18], "word": [3, 9], "want": [3, 6, 7, 11, 12, 13, 14, 16, 19], "j": [3, 4, 5, 6, 7, 8, 10, 11, 13, 14, 15], "compact": [3, 8, 14], "form": [3, 8, 10, 11, 12, 13, 16], "written": [3, 5, 8, 14, 17, 18], "6": [3, 4, 10, 14, 15, 16], "geometr": [3, 4, 5, 7, 16], "l": [3, 7, 8, 10, 14], "To": [3, 4, 7, 8, 11, 12, 13, 14, 15, 16], "comput": [3, 4, 5, 7, 9, 12, 13, 14, 15, 16], "sum_": [3, 6, 8, 10], "thi": [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19], "show": [3, 7, 11, 12, 17, 18], "obtain": [3, 4, 6, 7, 11, 14, 15, 16], "sum": [3, 8, 14, 15, 16], "each": [3, 8, 9, 10, 11, 12, 14, 15], "contribut": [3, 8], "singl": [3, 8, 10, 12], "all": [3, 4, 8, 10, 11, 12, 14, 15, 16, 17, 18], "still": [3, 9, 11, 14], "text": [3, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 17, 18], "partit": [3, 4], "column": [3, 6, 8, 14, 16], "vector": [3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15], "cdot": [3, 7], "label": [3, 8, 9, 11, 12, 13, 16], "c2": 3, "l1": 3, "equ5": 3, "case": [3, 4, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16], "14pt": 3, "allow": [3, 4, 6, 8, 9, 10, 13, 15, 17], "simpl": [3, 16, 17], "systemat": [3, 8, 15], "wai": [3, 8], "basi": [3, 16], "fact": [3, 4, 13, 15], "variabl": [3, 6, 8, 10, 11, 13, 15, 16], "particular": [3, 4, 6, 7, 10, 12, 13, 16], "third": [3, 4, 15], "2": [3, 4, 5, 6, 9, 10, 11, 12, 13, 14, 15, 16, 18], "select": 3, "first": [3, 4, 6, 7, 8, 9, 10, 12, 13, 14, 15, 16], "three": [3, 10, 14, 15], "element": [3, 8, 9, 11, 14, 16], "fourth": [3, 15, 16], "widetild": [3, 14], "4": [3, 4, 9, 10, 14, 15, 16, 18, 19], "llll": 3, "link": [3, 4, 10, 15], "planar": [3, 10, 15], "variou": [3, 10], "quad": [3, 4, 5, 6, 7, 8, 11, 12, 13, 14, 15, 16], "a_": [3, 4, 8, 15, 16], "c_": [3, 4, 7, 8, 10, 11, 14, 15, 16], "s_": [3, 4, 7, 8, 14, 15, 16], "12": [3, 4, 5, 8, 14, 15, 16], "123": [3, 15], "while": [3, 4, 9, 11, 12, 14, 15, 16], "unit": [3, 4, 8, 14, 15], "ax": [3, 8, 10, 14, 15, 16, 19], "thei": [3, 7, 10, 16, 17], "parallel": [3, 10, 14, 15, 16], "axi": [3, 4, 6, 8, 10, 15, 16], "z_": [3, 4, 5, 8, 10, 14, 15], "anthropomorph": [3, 4, 6, 10, 15], "gather": [3, 4, 8, 11, 14], "23": [3, 4, 14, 15, 16], "stanford": [3, 4, 15], "cccccc": 3, "5": [3, 4, 10, 15, 16, 19], "d_": [3, 10, 15, 16], "pose": [3, 7, 13, 14, 15, 16], "specifi": [3, 6, 7, 9, 10, 13, 15, 16], "minim": [3, 7], "number": [3, 10], "paramet": [3, 11, 14], "oper": [3, 4, 7, 9, 12, 14, 15, 16], "space": [3, 4, 5, 6, 7, 9, 10, 11, 12, 14, 15, 16], "sai": [3, 7, 9, 12, 13, 14], "phi": [3, 7, 14, 16], "do": [3, 4, 5, 6, 7, 9, 11, 12, 14, 17, 19], "so": [3, 4, 5, 6, 7, 11, 12, 14, 15, 16, 18], "mai": [3, 4, 7, 12, 15, 16], "know": 3, "alreadi": 3, "phi_": [3, 7], "out": [3, 4, 11, 14, 16, 19], "euler": [3, 16], "zyz": [3, 7, 14, 16], "angl": [3, 9, 10, 11, 15, 16], "varphi": [3, 7, 14], "psi": [3, 14], "correspond": [3, 4, 6, 7, 9, 11, 12, 14, 15, 16], "been": [3, 8, 11, 12, 14], "current": [3, 6, 7, 8, 9, 11, 12, 13], "illustr": [3, 4, 7, 12, 16], "compon": [3, 8, 12], "about": [3, 4, 6, 8, 12, 14, 15, 16, 17, 18, 19], "composit": 3, "elementari": [3, 5, 8], "thu": [3, 4, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16], "takeawai": 3, "viewpoint": [3, 6, 10], "more": [3, 4, 6, 15, 16, 18, 19], "intuit": [3, 16], "than": [3, 6, 9, 15], "instead": [3, 4, 6, 7, 14], "nonorthogon": 3, "orient": [3, 10, 14, 15, 16], "On": [3, 5, 8, 15], "integr": [3, 11], "over": [3, 7], "doe": [3, 7, 8, 9, 14], "admit": 3, "clear": 3, "seen": 3, "linear": [4, 5, 8, 12, 13], "ll": [4, 8, 17], "configur": [4, 5, 6, 8, 12, 13, 16], "those": [4, 9, 11, 16, 17], "rank": [4, 6, 7], "defici": 4, "why": [4, 8, 14, 15], "pai": 4, "attent": 4, "mobil": [4, 10], "reduc": [4, 8, 11, 12], "b": [4, 9, 10, 11, 12, 13, 15], "infinit": [4, 6, 16], "solut": [4, 6, 7, 16], "ik": 4, "exist": [4, 7, 16], "Near": 4, "small": [4, 6, 7, 9, 17], "caus": [4, 6, 10], "larg": [4, 6, 9, 10, 11, 12], "via": [4, 7, 15], "determin": [4, 5, 9, 10, 11, 12, 13, 14, 15, 16], "tediou": 4, "easi": [4, 7, 14], "complex": [4, 9, 11, 16], "structur": [4, 5, 9, 10, 16, 17], "For": [4, 8, 10, 11, 12, 13, 15, 16, 17, 19], "spheric": [4, 10, 15], "possibl": [4, 5, 6, 8, 10, 11, 14, 15, 16], "split": 4, "problem": [4, 6, 7, 12, 16], "separ": [4, 16], "result": [4, 7, 8, 12, 13, 14, 15], "outer": 4, "g": [4, 8, 9, 11, 12, 13, 16], "block": [4, 7, 8, 12, 13, 18], "11": [4, 8, 10, 14], "21": [4, 8, 13, 14], "22": [4, 8, 14], "By": [4, 5, 6, 7, 8, 11, 13], "conduct": 4, "row": [4, 6], "maintain": 4, "bar": [4, 6, 10], "operatornam": [4, 6, 8, 10, 12, 14, 16], "det": [4, 6, 14], "inspect": 4, "whenev": 4, "linearli": [4, 6], "depend": [4, 6, 7, 8, 9, 10, 12, 15, 16, 17], "reveal": [4, 8], "occur": [4, 14], "vartheta_": [4, 8, 15, 16], "loss": 4, "orthogon": [4, 10, 14], "characterist": [4, 11], "elbow": [4, 15, 16], "shoulder": [4, 6, 15, 16], "vanish": 4, "outstretch": 4, "retract": 4, "motion": [4, 6, 7, 9, 10, 11, 12, 13, 15, 16], "along": [4, 6, 7, 8, 15], "perpendicular": 4, "_2": 4, "_0": [4, 12, 13], "li": 4, "p_": [4, 14, 16], "start": [4, 17, 18], "describ": [4, 8, 10, 13, 14, 15, 16], "meant": 4, "concern": [4, 8], "task": [4, 9, 10, 15], "extract": 4, "rang": [4, 5, 6, 19], "subspac": [4, 5], "mathcal": [4, 5, 6, 7, 8, 14], "subseteq": 4, "mathbb": [4, 5, 6], "postur": [4, 5, 6, 12, 16], "null": [4, 5, 6], "produc": 4, "ani": [4, 5, 6, 8, 11, 12, 13, 14, 18], "full": [4, 6, 7], "one": [4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 17], "dim": 4, "span": [4, 17], "degener": 4, "dimens": [4, 15], "decreas": [4, 12, 13], "increas": [4, 11], "independ": [4, 8, 9, 10, 12, 14], "neq": [4, 7, 12, 13, 14], "emptyset": [4, 7], "project": [4, 6, 14], "equiv": [4, 5], "arbitrari": [4, 7, 15], "import": [4, 16, 19], "resolut": 4, "choos": [4, 7, 12, 13, 15], "exploit": [4, 6], "advantag": [4, 12], "appli": [5, 8, 9, 11, 14, 16], "torqu": [5, 8, 10, 11, 12, 13], "equilibrium": [5, 12, 13], "tau": [5, 8, 9, 11, 12], "gamma": [5, 14], "virtual": [5, 9], "work": [5, 9, 19], "w_": 5, "As": [5, 8, 11, 14, 16, 19], "f": [5, 7, 8, 9, 11, 13], "mu": 5, "moment": [5, 8], "displac": [5, 9], "angular": [5, 8], "onli": [5, 6, 7, 8, 9, 10, 12, 14, 15, 16], "delta": [5, 7, 11], "foral": [5, 7, 12, 13], "establish": [5, 6, 8, 9, 11, 15], "transpos": 5, "balanc": 5, "requir": [5, 6, 7, 8, 9, 11, 12], "worth": [5, 8, 14], "remark": [5, 14], "gamma_": 5, "absorb": 5, "constraint": [5, 10, 14], "reaction": 5, "them": 5, "exactli": [5, 7, 10, 12], "singular": [5, 7, 13], "remain": 5, "whatev": 5, "perp": 5, "onc": [5, 6, 8, 10, 15], "known": [5, 10], "complet": [5, 10, 15], "same": [5, 6, 14, 17], "rigid": [5, 8, 9, 10, 14, 15], "bodi": [5, 10, 14, 15], "concept": [5, 14, 15], "o_": [5, 14, 15], "x_": [5, 10, 14, 15, 16], "y_": [5, 10, 14, 15], "attach": [5, 8, 10, 14, 15], "compactli": [5, 14], "own": [5, 8, 14], "virtu": 5, "final": [5, 6, 8, 10, 15, 16], "analysi": [5, 11], "instantan": 5, "necessari": [5, 7, 8, 16], "recomput": 5, "suppos": [6, 11, 14, 16], "assign": [6, 16], "corn": 6, "inv": [6, 7], "resort": [6, 16], "pick": [6, 14], "best": 6, "some": [6, 10, 14, 15, 16, 17, 19], "sens": [6, 8, 10], "min_": [6, 7], "frac": [6, 7, 8, 11, 12, 13, 14, 16], "w": [6, 14, 16], "simplifi": [6, 15], "dagger": [6, 7], "pseudo": 6, "criteria": 6, "distanc": [6, 8, 15], "after": [6, 16], "mention": 6, "previou": [6, 7, 8, 9, 11, 12, 13, 14, 15], "lectur": [6, 7, 8, 9, 11, 12, 13, 16], "question": [6, 12, 15], "typic": [6, 7, 8, 9, 10, 11, 15, 16], "choic": [6, 7, 11, 12, 13, 16], "k_": [6, 8, 11], "partial": [6, 8], "secondari": 6, "object": [6, 7, 15], "gradient": 6, "attempt": 6, "maxim": 6, "local": [6, 9], "Such": [6, 14, 15], "measur": [6, 9, 13, 15], "sqrt": [6, 11, 14, 16], "awai": 6, "limit": [6, 10, 12, 15, 16], "maximum": [6, 15], "minimum": [6, 10, 15], "middl": 6, "keep": [6, 19], "center": [6, 8, 10, 15], "obstacl": [6, 15], "min": 6, "suitabl": [6, 8, 11], "nonsingular": 6, "At": [6, 7, 8, 15], "contain": [6, 10], "situat": 6, "even": 6, "though": [6, 15], "notin": 6, "cannot": [6, 7], "seriou": 6, "inconveni": 6, "neighborhood": 6, "rel": [6, 8, 10, 15], "exampl": [6, 9, 10, 17, 19], "path": [6, 11, 18], "pass": [6, 11], "nearbi": 6, "forc": [6, 8, 9, 10, 11, 12, 13], "make": [6, 15, 19], "short": 6, "track": [6, 7, 9, 11, 12, 13], "impos": 6, "trajectori": [6, 7, 12], "overcom": 6, "invert": 6, "differentia": 6, "neighbourhood": 6, "damp": [6, 11, 12], "least": [6, 15], "squar": [6, 7], "dl": 6, "star": 6, "k": [6, 7, 8, 9, 10, 11, 12, 13], "factor": [6, 11], "render": [6, 17], "better": 6, "condit": [6, 12, 13, 16], "numer": 6, "desir": [7, 9, 12, 13], "valu": [7, 8, 9, 12, 13, 15], "_d": [7, 12, 13], "forward": [7, 11, 13, 16], "solv": [7, 9, 12, 16], "ordinari": 7, "profil": 7, "view": [7, 8, 10, 15, 16], "input": [7, 9, 11, 12, 13, 17], "feedback": [7, 9, 13], "law": [7, 12], "plug": [7, 12, 13], "rightarrow": [7, 11, 12, 13, 14], "infti": [7, 11, 12, 13], "non": [7, 11, 13], "redund": [7, 13, 16], "verifi": [7, 14], "usual": [7, 9, 11, 12], "diagon": [7, 8, 9, 11], "asymptot": [7, 11, 12, 13], "stabl": [7, 12], "tend": [7, 11], "zero": [7, 12, 13], "converg": [7, 12, 13], "rate": [7, 13], "eigenvalu": 7, "od": 7, "bmatrix": [7, 12, 13], "lambda_1": 7, "lambda_2": 7, "ddot": [7, 8, 9, 11, 12, 13], "lambda_r": 7, "lambda_1t": 7, "lambda_2t": 7, "lambda_rt": 7, "therefor": [7, 8, 9, 10, 12, 14, 16], "scheme": [7, 9, 12, 13], "indic": 7, "nonlinear": [7, 12, 16], "assum": [7, 8, 9], "initi": [7, 12, 13, 14], "guess": 7, "_t": 7, "approxim": [7, 8], "_k": 7, "tyler": 7, "approx": [7, 9], "updat": 7, "step": [7, 14, 15, 16], "size": [7, 15], "befor": 7, "discret": 7, "dv": 7, "dt": [7, 8], "obvious": [7, 16], "neg": [7, 12, 13, 16], "under": [7, 9, 12, 13, 16], "assumpt": [7, 8, 9, 12, 16], "impli": [7, 11], "stabliti": 7, "descent": 7, "alpha": [7, 14, 16], "bigg": 7, "rvert_": 7, "semi": 7, "get": [7, 17, 18], "stuck": 7, "would": [7, 8], "suffici": 7, "introduc": [7, 8, 14], "section": [7, 9, 12], "furthermor": 7, "comment": 7, "becaus": [7, 15], "although": 7, "guarante": [7, 9], "norm": 7, "bound": [7, 10], "prevail": 7, "quadrat": [7, 8, 12, 13], "otherwis": [7, 14, 15], "domin": 7, "drag": 7, "analyt": 7, "natur": [7, 11, 12, 16], "howev": [7, 9, 11], "aris": [7, 15], "set": [7, 8, 10, 12, 13, 14, 15, 16], "wa": 7, "sin": [7, 14], "ensur": [7, 12], "eta_": [7, 14], "epsilon": [7, 14], "eta": [7, 14], "prove": 7, "qquad": [7, 8, 11, 14], "convert": [7, 18], "equival": [7, 11, 14], "acceler": [7, 8, 12, 13], "rout": 7, "matric": [7, 12, 13, 14, 16], "speed": [7, 12], "chosen": [7, 8, 10, 11, 15], "effect": [8, 11, 12], "lagrangian": 8, "u": [8, 10, 12, 13], "xi_": 8, "associ": [8, 15], "xi_i": 8, "should": [8, 10, 14, 18], "dual": 8, "q_i": [8, 15], "power": [8, 17], "xi": 8, "chain": [8, 10, 15], "net": 8, "actuat": [8, 10, 12, 15], "friction": [8, 9, 11, 12, 13], "well": [8, 10, 16, 19], "induc": 8, "environ": [8, 10], "due": [8, 9, 11, 12, 14], "ell_": 8, "m_": 8, "descript": [8, 15], "equ": [8, 9, 11, 12, 13], "link_k": 8, "int_": 8, "v_": 8, "rho": 8, "densiti": 8, "particl": 8, "volum": [8, 10], "l_": 8, "mass": 8, "r_": [8, 14], "consequ": 8, "substitut": [8, 11, 12], "ref": [8, 9, 11, 12, 13, 16], "multipl": [8, 16], "cross": 8, "int": 8, "i_": 8, "inertia": [8, 11, 12], "tensor": 8, "centr": 8, "denavit": 8, "hartenberg": 8, "constant": [8, 9, 11, 12, 13, 15], "coincid": [8, 14, 15], "central": [8, 13], "jmath": 8, "taken": 8, "account": 8, "up": [8, 10, 16], "llllll": 8, "locat": [8, 10, 15, 16], "formal": [8, 14, 15], "analog": [8, 13], "rotari": 8, "electr": [8, 10], "transmiss": [8, 11], "stator": 8, "sole": [8, 16], "rotor": [8, 9], "gear": [8, 9, 11], "reduct": 8, "ratio": [8, 11, 12], "dimension": [8, 10], "quantiti": [8, 11], "whose": [8, 10, 11, 12, 13], "b_": 8, "definit": [8, 10, 12, 13, 14, 15], "done": [8, 14], "store": [8, 17], "gravit": [8, 12], "graviti": 8, "vertic": 8, "regard": [8, 10], "detail": [8, 12, 18], "further": [8, 9, 11, 13, 15], "g_": 8, "h_": 8, "coeffici": 8, "centrifug": 8, "corioli": 8, "presenc": [8, 18], "underbrac": [8, 10, 11, 12], "viscou": 8, "sgn": [8, 14], "coulomb": 8, "h": [8, 9, 10, 11], "sign": [8, 9, 16, 19], "exert": 8, "summari": [8, 10], "satisfi": [8, 9, 16], "big": 8, "ijk": 8, "proof": 8, "idea": [8, 12], "equal": 8, "diff_kinenergi": 8, "extern": [8, 9, 12, 13, 15], "power_forc": 8, "_v": 8, "_s": 8, "intermedi": [8, 15], "previous": [8, 11, 15, 16], "multipli": 8, "subtract": 8, "wo": 8, "With": [8, 12, 13, 15, 18], "j_": 8, "scalar": [8, 14], "now": [8, 9, 14], "configurationdepend": 8, "off": [8, 14, 17, 18], "preced": 8, "especi": 8, "high": [8, 9, 11], "could": [8, 9], "neglect": 8, "appear": 8, "uniqu": [8, 15], "christoffel": 8, "symbol": 8, "111": 8, "112": 8, "121": 8, "122": 8, "211": 8, "212": 8, "221": 8, "222": 8, "7": [8, 10, 11, 15], "47": 8, "verif": 8, "absenc": 8, "tip": 8, "tau_": [8, 11], "sequenc": [9, 10, 14], "action": [9, 12], "execut": [9, 15, 18], "transient": 9, "steadi": 9, "error": [9, 12, 13, 15], "mani": [9, 15, 17, 18], "wherea": [9, 17], "made": [9, 14, 15], "There": [9, 10, 16, 19], "architectur": 9, "close": [9, 10, 11, 16], "loop": [9, 10, 11], "fig": [9, 10, 11, 12, 13, 14, 15, 19], "joint_control": 9, "design": [9, 11, 12, 13], "actual": [9, 12, 15], "regul": 9, "fashion": 9, "less": 9, "accuraci": [9, 15], "uncertainti": 9, "construct": [9, 10], "toler": 9, "lack": 9, "calibr": 9, "backlash": 9, "elast": 9, "operational_space_control": 9, "embed": 9, "fed": 9, "back": [9, 11, 14], "address": [9, 11, 12], "drawback": 9, "greater": [9, 15], "challeng": 9, "vision": 9, "without": [9, 11, 12, 13, 15], "difficult": 9, "accur": 9, "transmission_model": 9, "much": 9, "ident": 9, "drive": [9, 12], "transmission_model2": 9, "dc_model": 9, "_m": [9, 11], "armatur": [9, 11], "voltag": [9, 11], "resist": [9, 11], "emf": [9, 11], "amplifi": [9, 11], "servomotor": [9, 11], "voltage_control": 9, "unveil": 9, "_c": 9, "overal": 9, "diagram": [9, 12, 13], "uniti": [9, 14], "veri": [9, 14], "too": [9, 11], "whole": 9, "plu": 9, "proport": [9, 11, 12], "decentr": 9, "th": 9, "nice": 9, "current_torqu": 9, "_r": 9, "insid": 9, "_a": [9, 13], "voltage_curr": 9, "franka": [10, 15], "research": [10, 15], "connect": [10, 15, 16], "consecut": [10, 15, 16], "hydraul": 10, "pneumat": 10, "deliv": 10, "hydral": 10, "gripper": [10, 15], "barrett": 10, "shadow": 10, "serial": 10, "topolog": 10, "altern": 10, "everi": 10, "shape": 10, "door": 10, "hing": 10, "theta": [10, 11], "plane": [10, 11, 15], "coin": 10, "ly": 10, "head": 10, "flat": 10, "tabl": 10, "abraham": [], "lincoln": [], "real": 11, "3d": [10, 14], "8": 10, "c1": 16, "3dcoin": [], "place": 10, "freeli": 10, "subject": 10, "must": [10, 13, 16, 17], "sphere": 10, "radiu": 10, "latitud": 10, "longitud": 10, "circl": 10, "parametr": 10, "add": 10, "six": [10, 16], "noncollinear": 10, "spatial": [], "anoth": [10, 14, 15], "summar": [10, 15], "count": 10, "helic": 10, "cylindr": 10, "consist": 10, "ground": [10, 15], "f_": 10, "lower": [10, 16], "grubler": 10, "machen": 10, "four": [10, 15, 16], "linkag": 10, "slider": 10, "crank": 10, "five": 10, "stephenson": 10, "watt": 10, "overlap": 10, "meet": 10, "rather": 10, "correctli": [], "eight": 10, "9": [10, 14], "stewart": 10, "gough": 10, "platform": 10, "stationari": 12, "upper": 10, "leg": 10, "total": [10, 14, 15], "14": 10, "18": [10, 12, 13, 14], "surfac": [], "unlik": 14, "larger": [], "wrap": [], "around": [], "matter": [], "oval": [], "american": [], "footbal": [], "similarli": 14, "kind": 17, "continu": 11, "deform": [], "cut": [], "glu": [], "simpli": 14, "stretch": [], "turn": 14, "90": [], "circ": [], "south": [], "pole": 11, "north": [], "180": [], "edg": [], "arrow": [], "togeth": [], "toru": [], "mark": [], "parameter": [], "explicit": [], "valid": 16, "earth": [], "west": [], "implicit": [], "euclidean": [], "just": 17, "higher": 11, "portion": [], "most": [16, 17], "common": [14, 15, 16], "cartesian": [10, 15], "mutual": [10, 14], "replac": [10, 12, 13], "2nd": 10, "special": [10, 16, 17], "scara": 10, "realiz": [], "dispos": [], "dyn": 11, "theta_": [11, 15], "m1": 11, "m2": 11, "mn": 11, "dm": 11, "motor": 11, "motor_angl": 11, "diag": [11, 12], "r_1": 11, "r_2": 11, "r_n": 11, "r_i": 11, "motor_torqu": 11, "dyn2": 11, "analyz": 11, "bmat": 11, "overlin": 11, "averag": 11, "decouple_model": 11, "don": 11, "output": [11, 12, 13, 18], "single_joint_dyn": 11, "tau_m": 11, "individu": 11, "coupl": [11, 12], "prevent": 11, "disturb": [11, 12, 15], "omit": 11, "index": 11, "19": [11, 14], "i_m": 11, "k_t": 11, "r_a": 11, "dr_a": 11, "k_ti_a": 11, "v_a": 11, "k_v": 11, "g_vv_c": 11, "g_v": 11, "v_c": 11, "notat": [11, 14], "simplic": 11, "treat": [11, 18], "servo": 11, "But": [11, 19], "dc": 11, "itself": [11, 14], "theta_m": 11, "transfer": 11, "motortf": 11, "k_m": 11, "t_m": 11, "r_ai_m": 11, "eas": 11, "reader": 11, "theta_r": 11, "plant": 11, "backward": 11, "sensor": 11, "abl": 11, "littl": 11, "respons": [11, 13], "root": 11, "signal": [11, 12, 13], "laplac": 11, "lim_": 11, "sf": 11, "motor_model2": 11, "t_": 11, "k_p": 11, "t_p": 11, "tp": 11, "c_p": 11, "locu": 11, "plot": [11, 19], "gain": 11, "inher": 11, "unstabl": 11, "absolut": 11, "toward": 11, "faster": 11, "st_m": 11, "zeta": 11, "frequenc": [11, 12], "pm": [11, 16], "These": [11, 12, 15], "ones": 11, "ramp": 11, "vt": 11, "cancel": [11, 12], "reject": 11, "order": [11, 12, 13, 14, 15, 16], "help": [11, 15, 17], "unaccept": 11, "oscil": 11, "t_v": 11, "tv": 11, "pv_control": 11, "confin": 11, "region": 11, "recogn": 11, "transduc": 11, "k_mk_vk_": 11, "k_mk_pk_vk_": 11, "act": 12, "strongli": 12, "influenc": 12, "perform": [12, 15], "advis": 12, "knowledg": 12, "textbf": [12, 13], "manipulator_control": [12, 13], "eventu": [12, 13], "reach": [12, 13, 15], "lyapunov": [12, 13], "see": [12, 13, 17, 18, 19], "background": [12, 13], "16": [12, 13], "17": [12, 13], "candid": [12, 13], "lypuanovdot": 12, "controller2": 12, "long": [12, 13], "rigor": 12, "global": 12, "stabil": 12, "line": [12, 13, 17, 18, 19], "good": 12, "chang": [12, 13], "joint_sign": 12, "try": 12, "new": [12, 13, 14, 15], "yet": [12, 13], "dyn_new": [12, 13], "again": [12, 13, 16], "decoupl": [12, 16], "exact": 12, "error_dyn2": 12, "error_dyn": 12, "standard": 12, "_p": 12, "particularli": 12, "kpkd": 12, "zeta_": 12, "subsystem": 12, "my": [12, 13], "zeta_i": 12, "ni": 12, "affect": 12, "nonetheless": 12, "techniqu": 12, "perfect": 12, "quit": 12, "rais": 12, "sensit": 12, "unavoid": 12, "imperfect": 12, "_e": 13, "end_pose_error": 13, "That": [13, 14, 18], "similar": [13, 14, 17], "dotjacobian": 13, "nonredund": [13, 15, 16], "suggest": 13, "ysignal": 13, "addit": [13, 15], "versu": 13, "report": [13, 15], "44": 13, "besid": 13, "indirect": 13, "rbpo": [], "orthonorm": [], "counter": 14, "clockwis": 14, "rotation1": [], "co": [14, 16], "beta": [14, 16], "perspect": 14, "thought": 14, "postmultipl": [14, 15], "rule": [14, 15], "postmultipc": 14, "_1": 14, "premultipl": 14, "commut": 14, "bring": 14, "fulli": 14, "success": 14, "proper": [], "name": [14, 15], "rpy": 14, "roll": 14, "pitchyaw": [], "13": 14, "31": 14, "32": 14, "33": 14, "atan": [14, 16], "pitch": 14, "yaw": 14, "zyx": 14, "implement": [], "creat": [14, 19], "x_1": 14, "y_1": 14, "z_1": 14, "geometri": 15, "undefin": 14, "suffer": [], "disadvantag": [], "encount": [], "epsilon_": 14, "geq": 14, "product": 14, "belong": [], "group": [], "se": [], "what": [], "compos": [14, 15], "mount": 15, "tool": [15, 17], "degre": [], "freedom": [], "cccc": [15, 16], "normal": 15, "slide": 15, "jaw": 15, "convention": 15, "recurs": 15, "answer": [], "l2": 16, "alpha_": 15, "go": [], "nonuniqu": 15, "arbitrarili": 15, "procedur": [14, 15], "instanc": 15, "wrist": 15, "reachabl": [15, 16], "dexter": 15, "latter": 15, "former": 15, "leq": [15, 16], "sheet": 15, "manufactur": 15, "top": [15, 18], "nomin": 15, "deviat": [], "attain": [], "millimet": [], "list": [], "industri": [], "repeat": 15, "return": 15, "metric": [], "smaller": 15, "mathrm": [15, 16], "mm": 15, "02": 15, "intrins": 15, "anyhow": [], "endeffector": [], "lost": [], "lasercut": [], "irrelev": [], "spontan": [], "intention": 15, "util": 15, "versatil": 15, "constitut": 16, "seven": 15, "finger": 15, "thank": 15, "avoid": 15, "might": [15, 16], "prescrib": [], "craig": 15, "imag": [15, 19], "wiki": 15, "proxim": 15, "john": 15, "introduct": 15, "3rd": 15, "edit": 15, "classic": 15, "distal": [], "put": 15, "t_i": [], "rot": 15, "tran": 15, "emika": 15, "panda": 15, "increasingli": 15, "popular": 15, "teach": 15, "frankaemika": 15, "doc": 15, "control_paramet": 15, "html": [15, 19], "great": 15, "tutori": 15, "python": 15, "jhavl": 15, "dkt": 15, "peter": 15, "cork": 15, "alwai": 16, "admiss": 16, "talk": 16, "leav": 16, "identifi": [15, 16], "3link": 16, "cosin": 16, "theorem": 16, "triangl": 16, "segment": 16, "c_2": 16, "outsid": 16, "workspac": 16, "down": 16, "blow": 16, "preserv": 16, "partli": 16, "motiv": 16, "difficulti": 16, "like": [16, 17, 18], "inspir": 16, "itermedi": 16, "found": 16, "diment": 16, "sub": 16, "wx": 16, "wy": 16, "wz": 16, "iii": 16, "ii": [16, 19], "iv": 16, "s_3": 16, "atan2": 16, "elbowup": 16, "forearm": 16, "pair": 16, "compat": 16, "rrr": 16, "n_": 16, "whether": 17, "jupyt": [17, 18, 19], "notebook": 17, "ipynb": 17, "regular": 17, "md": [17, 18], "flavor": 17, "syntax": 17, "stand": 17, "markedli": 17, "slight": 17, "variat": 17, "commonmark": 17, "extens": 17, "sphinx": 17, "ecosystem": 17, "overview": 17, "markup": 17, "languag": 17, "serv": 17, "accept": 17, "box": 17, "build": 17, "inlin": 17, "document": [17, 18, 19], "cite": 17, "bibtex": 17, "holdgraf_evidence_2014": 17, "hdhpk14": 17, "moreov": 17, "insert": 17, "bibliographi": 17, "page": [17, 18], "properli": 17, "bib": 17, "christoph": 17, "ramsai": 17, "holdgraf": 17, "wendi": 17, "de": 17, "heer": 17, "brian": 17, "paslei": 17, "robert": 17, "knight": 17, "evid": 17, "predict": 17, "code": [17, 18], "auditori": 17, "cortex": 17, "intern": [15, 17], "confer": 17, "cognit": 17, "neurosci": 17, "brisban": 17, "australia": 17, "2014": 17, "frontier": 17, "starter": 17, "lot": [17, 19], "jupyterbook": 17, "org": 17, "instruct": 18, "print": 18, "built": 18, "default": 18, "kernel": 18, "displai": 18, "rest": 18, "jupytext": 18, "file": 18, "support": 18, "thing": 18, "inform": [18, 19], "run": 18, "command": 18, "init": 18, "markdownfil": 18, "emb": 19, "etc": 19, "post": 19, "add_": 19, "math": 19, "mbox": 19, "la_": 19, "tex": 19, "sure": 19, "escap": 19, "dollar": 19, "check": 19, "guid": 19, "sampl": 19, "matplotlib": 19, "rcparam": 19, "cycler": 19, "pyplot": 19, "plt": 19, "numpi": 19, "np": 19, "ion": 19, "contextlib": 19, "exitstack": 19, "0x1120e64d0": [], "random": 19, "reproduc": 19, "seed": 19, "19680801": 19, "10": 19, "logspac": 19, "100": 19, "randn": 19, "cmap": 19, "cm": 19, "coolwarm": 19, "prop_cycl": 19, "color": 19, "linspac": 19, "line2d": 19, "custom_lin": 19, "lw": 19, "subplot": 19, "figsiz": 19, "legend": 19, "cold": 19, "medium": 19, "hot": 19, "commonli": 10, "featur": 10, "numref": [], "pose_rigid_bodi": [], "20": 14, "boldsymbold": [], "ell": [], "fd_control2": [], "jpeg": [], "0x11db57a90": [], "0x113f43a10": 19, "did": [], "corrdin": 14, "0_1": 14, "1_2": 14, "primet": [], "happen": 14, "3dof": 14, "represnet": 14, "think": 14, "process": 14, "fighter": 14, "ject": [], "jet": 14, "taxi": 14, "fight": 14, "framework": 14, "zyz_euler_angl": [], "quanternion": 14, "easili": 14, "sequenti": 14, "tyipcal": 15, "24": 15, "fk": 15, "q_1": 15, "q_2": 15, "q_n": 15, "25": 15, "kick": [], "come": 15, "neatli": 15, "facili": 15, "basic": 15, "dh_convent": [], "last": 15, "assembl": 15, "calcul": 15, "righthand": 15, "d_i": 15, "theoric": 15, "nowdai": 15, "level": 15, "decent": 15, "1mm": 15, "care": 15, "_i": 15}, "objects": {}, "objtypes": {}, "objnames": {}, "titleterms": {"welcom": 0, "cours": [0, 7], "object": 0, "recommend": 0, "textbook": 0, "about": [0, 7], "thi": 0, "onlin": 0, "note": 0, "me": 0, "iri": 0, "lab": 0, "introduct": 1, "deriv": 2, "transform": [2, 5, 14], "pose": 2, "manipul": [2, 6, 9, 16], "link": [2, 8, 16], "veloc": [2, 5, 6, 11], "linear": [2, 3, 7], "angular": [2, 3], "summari": [2, 3], "jacobian": [3, 7], "exampl": [3, 8, 15, 18], "analyt": 3, "singular": [4, 6], "redund": [4, 6, 15], "decoupl": 4, "wrist": [4, 16], "arm": [4, 8, 10, 15, 16], "analysi": 4, "static": 5, "kineto": 5, "dualiti": 5, "forc": 5, "invers": [6, 7, 12, 13, 16], "kinemat": [6, 7, 14, 15, 16], "non": 6, "minim": [6, 11], "norm": 6, "selction": 6, "close": 6, "refer": 6, "numer": 7, "control": [7, 9, 11, 12, 13], "1": 7, "pseudo": 7, "newton": 7, "raphson": 7, "method": 7, "background": [7, 11], "from": 7, "system": [7, 9], "anoth": 7, "perspect": 7, "look": 7, "abov": 7, "2": [7, 8], "transpos": 7, "gradient": 7, "base": 7, "lyapunov": 7, "stabil": 7, "more": [7, 9, 17], "discuss": 7, "definit": 7, "orient": 7, "error": 7, "euler": [7, 14], "angl": [7, 14], "axi": [7, 14], "quaternion": [7, 14], "second": 7, "order": 7, "ik": [7, 16], "algorithm": 7, "dynam": [8, 12, 13], "lagrang": 8, "formul": 8, "comput": 8, "kinet": 8, "energi": 8, "i": [8, 12, 13], "motor": [8, 9], "total": 8, "potenti": 8, "equat": 8, "motion": 8, "properti": 8, "skew": 8, "symmetri": 8, "dot": 8, "boldsymbol": 8, "b": 8, "c": 8, "two": 8, "planar": [8, 16], "overview": 9, "robot": [9, 10, 15], "model": 9, "acutu": 9, "transmiss": 9, "between": 9, "actuat": 9, "joint": [9, 10, 11, 12], "electr": 9, "dc": 9, "an": [9, 18], "integr": 9, "torqu": 9, "gener": [9, 10], "configur": 10, "degre": 10, "freedom": 10, "rule": 10, "dof": 10, "differ": 10, "gr\u00fcbler": 10, "s": 10, "formula": 10, "space": 13, "workspac": [10, 15], "decentr": 11, "singl": 11, "diagram": 11, "final": 11, "valu": 11, "theorem": 11, "posit": 11, "feedback": 11, "central": 12, "pd": [12, 13], "graviti": [12, 13], "compens": [12, 13], "ii": [12, 13], "oper": 13, "basic": 14, "elementari": 14, "rotat": 14, "via": 14, "matrix": 14, "passiv": 14, "activ": 14, "composit": 14, "around": 14, "current": 14, "frame": 14, "fix": 14, "parameter": 14, "homogen": 14, "forward": 15, "denavit": 15, "hartenberg": 15, "dh": 15, "convent": 15, "modifi": 15, "paramet": 15, "three": 16, "spheric": 16, "anthropomorph": 16, "markdown": [17, 18, 19], "file": 17, "what": 17, "myst": [17, 18, 19], "sampl": 17, "role": 17, "direct": 17, "citat": 17, "learn": 17, "notebook": [18, 19], "cell": 18, "creat": 18, "quickli": 18, "add": 18, "yaml": 18, "metadata": 18, "content": 19, "code": 19, "block": 19, "output": 19, "type": 10, "figur": [], "lec19": [], "p_control2": [], "jpg": [], "width": [], "70": [], "name": [], "p_control_rl_1": [], "50": [], "p_control_rl_2": [], "k_": [], "tp": [], "m": [], "k_p": [], "t_p": [], "st_m": [], "represent": 14, "notat": [], "languag": [], "math": 15, "option": 15}, "envversion": {"sphinx.domains.c": 2, "sphinx.domains.changeset": 1, "sphinx.domains.citation": 1, "sphinx.domains.cpp": 6, "sphinx.domains.index": 1, "sphinx.domains.javascript": 2, "sphinx.domains.math": 2, "sphinx.domains.python": 3, "sphinx.domains.rst": 2, "sphinx.domains.std": 2, "sphinx.ext.intersphinx": 1, "sphinxcontrib.bibtex": 9, "sphinx": 56}})