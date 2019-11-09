%%
%% This is file `fei.cls',
%% generated with the docstrip utility.
%%
%% The original source files were:
%%
%% fei.dtx  (with options: `class')
%% -----------------------------------------------------------------------------------------------------
%% fei --- Class for the creation of academic works under the typographic rules of FEI University Center
%% Author: Douglas De Rizzo Meneghetti
%% E-mail: douglasrizzo@fei.edu.br
%%
%% Released under the LaTeX Project Public License v1.3c or later
%% See http://www.latex-project.org/lppl.txt
%% -----------------------------------------------------------------------------------------------------
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{fei}[2015/09/10 2.0 Modelo da FEI]
\LoadClass{report}
\DeclareOption{xindy}{
  \PassOptionsToPackage{\CurrentOption}{imakeidx}
  \PassOptionsToPackage{\CurrentOption}{glossaries}
}

\newif\ifrascunho
\rascunhotrue
\DeclareOption{rascunho}
{
\rascunhofalse
}

\newif\ifsublist
\sublistfalse
\DeclareOption{sublist}
{
  \sublisttrue
}

\newif\ifarial
\DeclareOption{arial}
{
  \arialtrue
}

\DeclareOption{times}
{
  \arialfalse
}

\newif\iftwoside
\twosidefalse
\DeclareOption{twoside}
{
  \twosidetrue
}

\DeclareOption{algo-as-figure}
{
  \PassOptionsToPackage{figure}{algorithm2e}
}










\ExecuteOptions{times} % processa opções-padrão

\ProcessOptions\relax % processa todas as opções

\renewcommand{\normalsize}{\fontsize{12pt}{14.4pt}\selectfont} % fonte do texto
\renewcommand{\footnotesize}{\fontsize{10pt}{12pt}\selectfont} % fonte das notas de rodapé
\RequirePackage[a4paper,top=3cm,bottom=2cm,left=3cm,right=2cm,bindingoffset=0cm,nofoot]{geometry}

\RequirePackage{fancyhdr}
\pagenumbering{arabic} % estilo da numeração das páginas
\pagestyle{fancy} % estilo dos cabeçalhos/rodapés
\fancyhf{}
\renewcommand{\footrulewidth}{0pt}
\renewcommand{\headrulewidth}{0pt}
\fancyhead[RO,LE]{\footnotesize\thepage}% numero da página em fonte menor que a do texto
\setlength{\headheight}{14.4pt}

\RequirePackage[english,portuges]{babel}
\RequirePackage[T1]{fontenc}

\RequirePackage{setspace}
\spacing{1.45}
\setlength{\parindent}{1.25cm} % recuo do paragrafo

\RequirePackage[font={singlespacing},format=hang, justification=raggedright,labelsep=endash,singlelinecheck=false]{caption} % fontes das legendas

\selectlanguage{portuges} % idioma do documento

\widowpenalty=10000
\clubpenalty=10000

\RequirePackage{mathtools} % matematica
\RequirePackage{lmodern} % Latin Modern, fontes tipográficas mais recentes que as do Knuth (Computer Modern)
\RequirePackage{icomma} % vírgula como separador decimal

\ifarial
  % \RequirePackage{helvet}
  \usepackage[scaled]{uarial}
  \renewcommand*\familydefault{\sfdefault} %% Only if the base font of the document is to be sans serif
\else
  \RequirePackage{times} % usar fonte times no texto todo
\fi

\RequirePackage{graphicx}      % figuras
\RequirePackage{morewrites}    % permite ao LaTeX escrever em mais de 16 arquivos auxiliares simultaneamente

\RequirePackage[plain,portuguese,algochapter,linesnumbered,inoutnumbered]{algorithm2e}
\SetKwInput{Entrada}{Entrada}
\SetKwInput{Saida}{Sa\'ida}
\SetKwInput{Dados}{Dados}
\SetKwInput{Resultado}{Resultado}
\SetKw{Ate}{at\'e}
\SetKw{Retorna}{retorna}
\SetKwBlock{Inicio}{in\'icio}{fim}
\SetKwIF{Se}{SenaoSe}{Senao}{se}{ent\~ao}{sen\~ao se}{sen\~ao}{fim se}
\SetKwSwitch{Selec}{Caso}{Outro}{selecione}{faça}{caso}{sen\~ao}{fim caso}{fim selec}
\SetKwFor{Para}{para}{fa\c{c}a}{fim para}
\SetKwFor{ParaPar}{para}{fa\c{c}a em paralelo}{fim para}
\SetKwFor{ParaCada}{para cada}{fa\c{c}a}{fim para cada}
\SetKwFor{ParaTodo}{para todo}{fa\c{c}a}{fim para todo}
\SetKwFor{Enqto}{enquanto}{fa\c{c}a}{fim enqto}
\SetKwRepeat{Repita}{repita}{at\'e}

\renewcommand{\@algocf@capt@plain}{above} % legenda acima
\SetAlgoCaptionSeparator{ --} % separador da legenda
\SetAlCapSty{} % estilo da primeira parte da legenda (remove negrito padrão)
\SetAlCapFnt{\normalsize} % fonte da primeira parte da legenda
\RequirePackage{amsthm,thmtools}
\renewcommand{\listtheoremname}{Lista de Teoremas} % traduz nome da lista de teoremas

\declaretheoremstyle[
spaceabove=6pt, spacebelow=6pt,
headfont=\normalfont\bfseries,
notefont=\normalfont\bfseries, notebraces={-- }{},
bodyfont=\normalfont,
postheadspace=1em
]{feistyle}

\declaretheorem[style=feistyle,name=Axioma]{axioma}
\declaretheorem[style=feistyle,name=Teorema]{teorema}
\declaretheorem[style=feistyle,name=Lema]{lema}
\declaretheorem[style=feistyle,name=Hip\'otese]{hipotese}
\declaretheorem[style=feistyle,name=Proposi\c{c}\~ao]{proposicao}
\declaretheorem[style=feistyle,name=Conjectura]{conjectura}
\declaretheorem[style=feistyle,name=Paradoxo]{paradoxo}
\declaretheorem[style=feistyle,name=Corol\'ario]{corolario}
\declaretheorem[style=feistyle,name=Defini\c{c}\~ao]{definicao}
\declaretheorem[style=feistyle,name=Exemplo]{exemplo}
\declaretheorem[style=remark,name=Demonstra\c{c}\~ao,qed=\qedsymbol,numbered=no]{prova}

\usepackage{chngcntr}
\counterwithout{figure}{chapter}
\counterwithout{table}{chapter}
\counterwithout{algocf}{chapter}
\counterwithout{equation}{chapter}

\renewcommand{\figurename}{\fontsize{10pt}{10pt}\selectfont Figura}
\renewcommand{\tablename}{\fontsize{10pt}{10pt}\selectfont Tabela}
\RequirePackage{enumitem}
\setlist[1]{align=left,labelindent=\parindent,labelwidth=*,labelsep=0.5em,label={\alph*)},ref=\theenumi}
\setlist[2]{align=left,labelindent=\parindent,labelwidth=*,labelsep=0.5em,label={--},ref=\theenumii}
\renewenvironment{itemize}{\begin{enumerate}}{\end{enumerate}} % troca o itemize pelo enumerate (seguindo o manual da biblioteca)

\renewcommand{\floatpagefraction}{.8} % página terá apenas floats se o float ocupar pelo menos 80% da página

\renewcommand{\part}{% não usado no texto, só para algumas páginas (resumo, abstract, agradecimentos...)
\@startsection{part}{-1}{0pt}{1\baselineskip}{1\baselineskip}{\cleardoublepage\fontsize{12pt}{14.4pt}\centering\bf\MakeUppercase}}

\renewcommand{\chapter}{\clearpage\pagestyle{fancy}%
\@startsection{chapter}{0}{0pt}{0pt}{1.0cm}{\fontsize{12pt}{14.4pt}\bfseries\MakeUppercase}}

\renewcommand{\section}{%
\@startsection{section}{1}{0pt}{\baselineskip}{\baselineskip}{\fontsize{12pt}{14.4pt}\MakeUppercase}}

\renewcommand{\subsection}{%
\@startsection{subsection}{2}{0pt}{\baselineskip}{\baselineskip}{\fontsize{12pt}{14.4pt}\bfseries}}

\renewcommand{\subsubsection}{%
\@startsection{subsubsection}{3}{0pt}{\baselineskip}{\baselineskip}{\fontsize{12pt}{14.4pt}\bfseries\itshape}}

\renewcommand{\paragraph}{%
\@startsection{paragraph}{4}{0pt}{\baselineskip}{\baselineskip}{\fontsize{12pt}{14.4pt}\it}}

\setcounter{secnumdepth}{4} % numerar divisões até o quarto nível (paragraph)
\setcounter{tocdepth}{4} % incluir divisões no sumário até o quarto nível (paragraph)

%% configuracao do sumario e listas de tabelas e figuras
\RequirePackage[titles]{tocloft} % para poder fazer mais coisas no sumario e nas listas

\tocloftpagestyle{empty} % remove numeração das páginas controladas pelo tocloft

\renewcommand{\cfttoctitlefont}{\hfil\bf\MakeUppercase}
\renewcommand{\cftloftitlefont}{\hfill\bf\MakeUppercase}
\renewcommand{\cftlottitlefont}{\hfill\bf\MakeUppercase}
\renewcommand{\cftafterloftitle}{\hfill}
\renewcommand{\cftafterlottitle}{\hfill}

\renewcommand{\cftchapindent}{0pt}
\renewcommand{\cftsecindent}{0pt}
\renewcommand{\cftsubsecindent}{0pt}
\renewcommand{\cftsubsubsecindent}{0pt}
\renewcommand{\cftparaindent}{0pt}

\renewcommand{\cftchapnumwidth}{\cftparanumwidth}
\renewcommand{\cftsecnumwidth}{\cftparanumwidth}
\renewcommand{\cftsubsecnumwidth}{\cftparanumwidth}
\renewcommand{\cftsubsubsecnumwidth}{\cftparanumwidth}
\renewcommand{\cftparanumwidth}{4em}

\renewcommand{\cftbeforechapskip}{0pt} % remove recuo antes de entradas de capítulos no sumário
\renewcommand{\cftchapfont}{\bfseries} % coloca o titulo de capítulos em negrito
\renewcommand{\cftchappagefont}{} % o número da página dos capítulos não é em negrito
\renewcommand{\cftsubsecfont}{\bfseries} % coloca o titulo das secoes em negrito
\renewcommand{\cftsubsubsecfont}{\bfseries\itshape} % coloca o titulo das secoes em negrito
\renewcommand{\cftparafont}{\itshape} % coloca o titulo das secoes em negrito
\renewcommand{\cftpartleader}{\cftdotfill{\cftdotsep}} % pontos no sumário para partes
\renewcommand{\cftchapleader}{\cftdotfill{\cftdotsep}} % pontos no sumário para capítulos

\setlength{\cftfignumwidth}{7.2em} % espaço onde a palavra "Ilustração" é escrita
\setlength{\cfttabnumwidth}{5.7em} % espaço onde a palavra "Tabela" é escrita
\renewcommand{\cftfigpresnum}{Figura } % escrita que precede cada entrada na lista de ilustrações
\renewcommand{\cfttabpresnum}{Tabela } % escrita que precede cada entrada na lista de tabelas
\renewcommand{\cftfigaftersnum}{ --} % traço na frente da escrita que precede as entradas na lista de ilustrações
\renewcommand{\cfttabaftersnum}{ --} % traço na frente da escrita que precede as entradas na lista de tabelas


\renewcommand{\tableofcontents}{\part*{\contentsname}\pagestyle{empty}\@starttoc{toc}\cleardoublepage}
\renewcommand{\listoftables}{\part*{\listtablename}\pagestyle{empty}\@starttoc{lot}\cleardoublepage}
\renewcommand{\listoffigures}{\part*{\listfigurename}\pagestyle{empty}\@starttoc{lof}\cleardoublepage}

\renewcommand{\listofalgorithms}{\begingroup%
\let\oldnumberline\numberline%
\renewcommand{\numberline}{Algoritmo~\oldnumberline}%
\part*{\listalgorithmcfname}\pagestyle{empty}\@starttoc{loa}\cleardoublepage\endgroup}

\renewcommand{\listoftheorems}{\begingroup%
\let\oldnumberline\numberline%
\renewcommand{\numberline}{Teorema~\oldnumberline}%
\part*{\listtheoremname}\thispagestyle{empty}\@starttoc{loe}\cleardoublepage\endgroup}

\def\and{\\} % modifica função do comando \and para ele ser usado na declaração de múltiplos autores

\renewcommand{\maketitle}{
\pagestyle{empty}
\begin{center}
\MakeUppercase{\@instituicao}\\[\baselineskip]
\uppercase\expandafter{\@author}
\textbf{
\vfill
\MakeUppercase{\@title}\ifthenelse{\isundefined{\@subtitulo}}{}{: \@subtitulo}
}
\vfill
\@cidade\\
\number\year%
\end{center}
\cleardoublepage
}


\newenvironment{folhaderosto}{
\setcounter{page}{1}
\thispagestyle{empty}
\begin{center}
\uppercase\expandafter{\@author}\\
\vspace*{0.45\textheight}
\textbf{\MakeUppercase{\@title}}\ifthenelse{\isundefined{\@subtitulo}}{}{: \@subtitulo}
\vfill
\begin{flushright}
\begin{singlespace}
\begin{minipage}{0.55\textwidth}}{\end{minipage}{}
\end{singlespace}
\end{flushright}
\vfill
\@cidade\\
\number\year
\end{center}%
\clearpage}

\RequirePackage{pdfpages}
\RequirePackage{ifthen}
\newcommand{\folhadeaprovacao}{
\ifrascunho
  \includepdf{ata.pdf}\cleardoublepage
\else
  \thispagestyle{empty}\mbox{}\vfill\begin{center}\begin{Huge}Folha de aprova\c{c}\~{a}o\end{Huge}\vfill\end{center}\cleardoublepage
\fi
}

\newcommand{\fichacatalografica}{
  \iftwoside
  \else
    % se não for frente e verso, a ficha catalográfica não é contada no verso da folha de rosto
    \addtocounter{page}{-1}
  \fi
\ifrascunho
\includepdf{ficha.pdf}\cleardoublepage
\else
  \thispagestyle{empty}\mbox{}\vfill\begin{center}\begin{Huge}Ficha catalogr\'{a}fica\end{Huge}\vfill\end{center}\cleardoublepage
\fi
}

\newcommand{\subtitulo}[1]{\def\@subtitulo{#1}}

\newcommand{\smallcaption}[1]{\captionsetup{font=small}\caption*{#1}}

\def\@cidade{S\~ao Bernardo do Campo}
\newcommand{\cidade}[1]{\def\@cidade{#1}}

\def\@instituicao{Centro Universit\'ario FEI}
\newcommand{\instituicao}[1]{\def\@instituicao{#1}}

\newcommand{\advisor}[1]{\def\@advisor{#1}}

\def\@curso{Coisa Nenhuma}
\newcommand{\curso}[1]{\def\@curso{#1}}

\newcommand{\dedicatoria}[1]{
\cleardoublepage
\thispagestyle{empty}
\begin{flushleft}
\vspace*{\fill}
\hspace*{0.4\paperwidth\relax}
\begin{minipage}[l]{0.5\textwidth}
#1
\end{minipage}
\end{flushleft}
}

\newcommand{\epigrafe}[2]{
\cleardoublepage
\thispagestyle{empty}
\begin{flushleft}
\vspace*{\fill}
\hspace*{0.4\paperwidth\relax}
\begin{minipage}[l]{0.5\textwidth}
``{#1}''\\#2
\end{minipage}
\end{flushleft}
}

\newenvironment{resumo}{\part*{Resumo}\pagestyle{empty}\begin{spacing}{1.1}\noindent\normalsize}{\end{spacing}\cleardoublepage\pagestyle{fancy}}

\renewenvironment{abstract}{\selectlanguage{english}\part*{Abstract}\pagestyle{empty}\begin{spacing}{1.1}\noindent\normalsize}{\end{spacing}\cleardoublepage\pagestyle{fancy}\selectlanguage{portuges}}

\newenvironment{agradecimentos}{\part*{Agradecimentos}\pagestyle{empty}}{\cleardoublepage\pagestyle{fancy}}

\RequirePackage{imakeidx}
\renewcommand{\indexname}{\'Indice}
\let\oldmakeindex\makeindex
\let\oldprintindex\printindex
\renewcommand{\makeindex}{\oldmakeindex[title=\hfill \'INDICE \hfill \mbox{}]}
\renewcommand{\printindex}{\addcontentsline{toc}{chapter}{\hspace{\cftsubsubsecnumwidth}\'INDICE}%
\renewcommand{\chapter}{%
\@startsection{chapter}{0}{0pt}{0pt}{1.5cm}{\clearpage\fontsize{12pt}{14.4pt}\bf\MakeUppercase}}%
\oldprintindex}%

\RequirePackage[pdftex,pdfborder={0 0 0},colorlinks={false}]{hyperref}

\ifsublist
  \RequirePackage[symbols,nomain,acronym,nonumberlist]{glossaries}
  % estilo usado como base
  \setglossarystyle{alttree}
  % Configuracao de identacao do nivel 0 (titulos)
  \glssetwidest[0]{}
  % Configuracao de identacao do nivel 1 (a lista de simbolos em si)
  \glssetwidest[1]{aaaaaaaaaaaa}

  % remove número de página das listas de símbolos e abreviaturas (executado na primeira página)
  \renewcommand*{\glossarypreamble}{\thispagestyle{empty}\pagestyle{empty}\vspace*{-2\baselineskip}}

\else
  \RequirePackage[symbols,nomain,acronym,nonumberlist,nogroupskip]{glossaries}

  \newglossarystyle{mylong}{%
    \setglossarystyle{long}% base this style on the long style
    \renewenvironment{theglossary}{%
      \begin{longtable*}{lp{\glsdescwidth}}}%
    {\end{longtable*}}%
  }%

  \setglossarystyle{mylong}
  \renewcommand{\glsnamefont}[1]{\textbf{#1}}
  \setlength\LTleft{0pt}
  \setlength\LTright{0pt}
  \setlength\glsdescwidth{\linewidth}

  % remove número de página das listas de símbolos e abreviaturas (executado na primeira página)
  \renewcommand*{\glossarypreamble}{\thispagestyle{empty}\pagestyle{empty}}
\fi

\addto\captionsportuges{%
  \renewcommand*{\listfigurename}{Lista de Figuras}%
  \renewcommand*{\contentsname}{Sum\'ario}%
  \renewcommand*{\acronymname}{\hfill Lista de Abreviaturas \hfill \mbox{}}%
  \renewcommand*{\glssymbolsgroupname}{\hfill Lista de S\'imbolos \hfill \mbox{}}}

\renewcommand*{\glsclearpage}{\pagestyle{empty}}
\renewcommand*{\glossarypostamble}{\pagestyle{empty}\cleardoublepage}

\newcommand{\palavraschave}[1]{\mbox{}\\\noindent Palavras-chave: #1}% o resumo pede palavras chave no final
\newcommand{\keywords}[1]{\mbox{}\\\noindent Keywords: #1}% mesma coisa, mas pro abstract

\renewcommand{\appendix}{%
\renewcommand{\chaptername}{\appendixname}%
\setcounter{chapter}{0}% zera o contador do capítulo
\renewcommand{\thechapter}{\Alph{chapter}}% deixa o contador do capítulo em alfabético
\renewcommand{\chapter}[1]{% redefine o comando do capítulo
\stepcounter{chapter}% soma 1 ao contador do capítulo
\clearpage\phantomsection\thispagestyle{empty}\mbox{}\vfill\begin{center}\MakeUppercase{\textbf{AP\^ENDICE \thechapter\ --} ##1}\end{center}\vfill% adiciona uma folha com a letra e título do apêndice
\addcontentsline{toc}{chapter}{\hspace{\cftparanumwidth}AP\^ENDICE \Alph{chapter} -- ##1}%
\newpage%
}%
}%

\newcommand{\anexos}{%
\renewcommand{\chaptername}{Anexo}%
\setcounter{chapter}{0}%
\renewcommand{\thechapter}{\Alph{chapter}}%
\renewcommand{\chapter}[1]{%
\stepcounter{chapter}%
\clearpage\phantomsection\thispagestyle{empty}\mbox{}\vfill\begin{center}\MakeUppercase{\textbf{ANEXO \thechapter\ --} ##1}\end{center}\vfill%
\phantomsection%
\addcontentsline{toc}{chapter}{\hspace{\cftparanumwidth}ANEXO \Alph{chapter} -- ##1}%
\newpage%
}%
}%


\RequirePackage[alf,abnt-emphasize=bf,abnt-repeated-author-omit=yes,abnt-thesis-year=title]{abntex2cite}


\newcommand*{\citefloat}[1]{\citeauthoronline{#1}, \citeyear{#1}}

\renewenvironment{quote}
               {\begin{singlespace}\list{}{%
               \fontsize{10pt}{1em}%
               \leftmargin=4cm}%
               \item\relax\ignorespaces}
               {\endlist\end{singlespace}}

\renewenvironment{quotation}
               {\begin{singlespace}\list{}{%
               \fontsize{10pt}{1em}%
               \leftmargin=2cm \rightmargin=2cm%
               \listparindent .5cm \itemindent}%
                \item\relax}
               {\endlist\end{singlespace}}

\newlength{\bibitemsep}\setlength{\bibitemsep}{18pt}
\newlength{\bibparskip}\setlength{\bibparskip}{0pt} % zera o espaço entre parágrafos
\let\oldthebibliography\thebibliography
\renewcommand\thebibliography[1]{%
  \oldthebibliography{#1}%
  \setlength{\parskip}{\bibparskip}%
  \setlength{\itemsep}{\bibitemsep}%
}

\let\oldbibliography\bibliography
\renewcommand{\bibliography}[1]{%
\renewcommand{\bibname}{\hfill Refer\^encias \hfill\mbox{}}% muda o nome do titulo (modelo da biblioteca)
\clearpage\phantomsection\addcontentsline{toc}{chapter}{\hspace{\cftparanumwidth}\mdseries REFER\^ENCIAS}% adiciona o titulo ao sumario
\oldbibliography{#1}% adiciona realmente a bibliografia
}
%%
%% Copyright (C) 2015 by Douglas De Rizzo Meneghetti <douglasrizzo@fei.edu.br>
%%
%% This work may be distributed and/or modified under the
%% conditions of the LaTeX Project Public License (LPPL), either
%% version 1.3c of this license or (at your option) any later
%% version.  The latest version of this license is in the file:
%%
%% http://www.latex-project.org/lppl.txt
%%
%% This work is "maintained" (as per LPPL maintenance status) by
%% Douglas De Rizzo Meneghetti.
%%
%% This work consists of the file  fei.dtx,
%% and the derived files           fei.pdf and
%%                                 fei.cls.
%%
%%
%% End of file `fei.cls'.
