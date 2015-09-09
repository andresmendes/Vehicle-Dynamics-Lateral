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
\ProvidesClass{fei}[2015/02/08 0.9.2 Modelo da FEI]
\LoadClass{report}
\DeclareOption{xindy}{
  \PassOptionsToPackage{\CurrentOption}{imakeidx}
  \PassOptionsToPackage{\CurrentOption}{glossaries}
}

\DeclareOption{rascunho}
{
\newif\iflogvar
}

\newif\ifsublist
\sublistfalse

\DeclareOption{sublist}
{
  \sublisttrue
}

\DeclareOption{final}
{
\newif\iflogvar
\logvartrue
}

\ExecuteOptions{final} % final é padrão

\ProcessOptions\relax % processa todas as opções

\setlength{\paperheight}{297mm}
\setlength{\paperwidth}{210mm}
\renewcommand{\normalsize}{\fontsize{12pt}{14.4pt}\selectfont} % fonte do texto
\renewcommand{\footnotesize}{\fontsize{10pt}{12pt}\selectfont} % fonte das notas de rodapé
\RequirePackage[top=3cm,bottom=2cm,left=3cm,right=2cm]{geometry}

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

\RequirePackage{mathtools}     % matematica
\RequirePackage{lmodern}       % Latin Modern, fontes tipográficas mais recentes que as do Knuth (Computer Modern)
\RequirePackage{times}         % Times New Roman como fonte de tudo
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

\SetAlgoCaptionSeparator{--} % separador da legenda
\SetAlCapFnt{\footnotesize} % fonte da legenda
\RequirePackage{amsthm,thmtools}
\renewcommand{\listtheoremname}{Lista de Teoremas} % traduz nome da lista de teoremas

\declaretheoremstyle[
spaceabove=6pt, spacebelow=6pt,
headfont=\normalfont\bfseries,
notefont=\normalfont\bfseries, notebraces={-- }{},
bodyfont=\normalfont,
postheadspace=1em,
qed=\qedsymbol
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
\renewenvironment{itemize}{\begin{enumerate}}{\end{enumerate}} % troca o itemize pelo enumerate (seguindo o manual da biblioteca)
\renewcommand{\theenumi}{\alph{enumi})}                        % deixa as listas com letras no lugar do numero
\renewcommand{\labelenumi}{\theenumi}                          % continuacao

 % troca a numeracao no segundo nivel para letras
\renewcommand{\theenumii}{--}
\renewcommand{\labelenumii}{\theenumii}

\renewcommand{\part}{% não usado no texto, só para algumas páginas (resumo, abstract, agradecimentos...)
\@startsection{part}{-1}{0pt}{2\baselineskip}{2\baselineskip}{\clearpage\fontsize{12pt}{14.4pt}\centering\bf\MakeUppercase}}

\renewcommand{\chapter}{\clearpage%
\@startsection{chapter}{0}{0pt}{0pt}{1.5cm}{\thispagestyle{fancy}\fontsize{12pt}{14.4pt}\bf\MakeUppercase}}

\renewcommand{\section}{%
\@startsection{section}{1}{0pt}{1.5\baselineskip}{1.5\baselineskip}{\fontsize{12pt}{14.4pt}\MakeUppercase}}

\renewcommand{\subsection}{%
\@startsection{subsection}{2}{0pt}{1.5\baselineskip}{1.5\baselineskip}{\fontsize{12pt}{14.4pt}\bf}}

\renewcommand{\subsubsection}{%
\@startsection{subsubsection}{3}{0pt}{1.5\baselineskip}{1.5\baselineskip}{\fontsize{12pt}{14.4pt}\it\bf}}

\renewcommand{\paragraph}{%
\@startsection{paragraph}{4}{0pt}{1.5\baselineskip}{1.5\baselineskip}{\fontsize{12pt}{14.4pt}\it}}

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

\renewcommand{\cftbeforechapskip}{0pt} % remove recuo antes de entradas de capítulos no sumário
\renewcommand{\cftchapfont}{\bfseries} % coloca o titulo de capítulos em negrito
\renewcommand{\cftchappagefont}{} % o número da página dos capítulos não é em negrito
\renewcommand{\cftsubsecfont}{\bfseries} % coloca o titulo das secoes em negrito
\renewcommand{\cftsubsubsecfont}{\bfseries\itshape} % coloca o titulo das secoes em negrito
\renewcommand{\cftparafont}{\itshape} % coloca o titulo das secoes em negrito
\renewcommand{\cftpartleader}{\cftdotfill{\cftdotsep}} % pontos no sumário para partes
\renewcommand{\cftchapleader}{\cftdotfill{\cftdotsep}} % pontos no sumário para capítulos

\setlength{\cftfignumwidth}{7em} % espaço onde a palavra "Ilustração" é escrita
\setlength{\cfttabnumwidth}{5.5em} % espaço onde a palavra "Tabela" é escrita
\renewcommand{\cftfigpresnum}{Ilustra\c{c}\~ao } % escrita que precede cada entrada na lista de ilustrações
\renewcommand{\cfttabpresnum}{Tabela } % escrita que precede cada entrada na lista de tabelas
\renewcommand{\cftfigaftersnum}{ --} % traço na frente da escrita que precede as entradas na lista de ilustrações
\renewcommand{\cfttabaftersnum}{ --} % traço na frente da escrita que precede as entradas na lista de tabelas


\addto\captionsportuges{%
  \renewcommand\listfigurename{Lista de Ilustra\c{c}\~oes}%
  \renewcommand\contentsname{Sum\'ario}}
\renewcommand{\tableofcontents}{\part*{\contentsname}\pagestyle{empty}\@starttoc{toc}\clearpage\pagestyle{fancy}}
\renewcommand{\listoftables}{\part*{\listtablename}\pagestyle{empty}\@starttoc{lot}\clearpage\pagestyle{fancy}}
\renewcommand{\listoffigures}{\part*{\listfigurename}\pagestyle{empty}\@starttoc{lof}\clearpage\pagestyle{fancy}}

\renewcommand{\listofalgorithms}{\begingroup%
\let\oldnumberline\numberline%
\renewcommand{\numberline}{Algoritmo~\oldnumberline}%
\part*{\listalgorithmcfname}\thispagestyle{empty}\@starttoc{loa}\endgroup}
\renewcommand{\listoftheorems}{\begingroup%
\let\oldnumberline\numberline%
\renewcommand{\numberline}{Teorema~\oldnumberline}%
\part*{\listtheoremname}\thispagestyle{empty}\@starttoc{loe}\endgroup}

\def\and{\\} % modifica função do comando \and para ele ser usado na declaração de múltiplos autores

\renewcommand{\maketitle}{
\clearpage
\thispagestyle{empty}
\begin{center}
\textbf{
\MakeUppercase{\@instituicao}\\[\baselineskip]
\uppercase\expandafter{\@author}
\vfill
\MakeUppercase{\@title}\ifthenelse{\isundefined{\@subtitulo}}{}{: \@subtitulo}}
\vfill
\@cidade\\
\number\year
\end{center}
}

\newenvironment{folhaderosto}{
\clearpage
\thispagestyle{empty}
\begin{center}
\uppercase\expandafter{\@author}\\
\vspace*{0.45\textheight}
\textbf{\MakeUppercase{\@title}}\ifthenelse{\isundefined{\@subtitulo}}{}{: \@subtitulo}
\vfill
\begin{flushright}
\begin{minipage}{0.55\textwidth}}{\end{minipage}{}
\end{flushright}
\vfill
\@cidade\\
\number\year
\end{center}}

\RequirePackage{pdfpages}
\RequirePackage{ifthen}
\newcommand{\folhadeaprovacao}{
\iflogvar
  \includepdf{ata.pdf}
\else
  \clearpage\thispagestyle{empty}\mbox{}\vfill\begin{center}\begin{Huge}Folha de aprova\c{c}\~{a}o\end{Huge}\vfill\end{center}
\fi
}

\newcommand{\fichacatalografica}{
    \addtocounter{page}{-1}
\iflogvar
\includepdf{ficha.pdf}
\else
\clearpage\thispagestyle{empty}\mbox{}\vfill\begin{center}\begin{Huge}Ficha catalogr\'{a}fica\end{Huge}\vfill\end{center}
\fi
}

\newcommand{\subtitulo}[1]{\def\@subtitulo{#1}}

\newcommand{\smallcaption}[1]{\captionsetup{font=small}\caption*{#1}}

\def\@cidade{S\~ao Bernardo do Campo}
\newcommand{\cidade}[1]{\def\@cidade{#1}}

\def\@instituicao{Centro Universit\'ario da FEI}
\newcommand{\instituicao}[1]{\def\@instituicao{#1}}

\newcommand{\dedicatoria}[1]{
\clearpage
\thispagestyle{empty}
\begin{flushleft}
\vspace*{0.5\paperheight\relax}
\hspace*{0.4\paperwidth\relax}
\begin{minipage}[l]{0.5\textwidth}
#1
\end{minipage}
\end{flushleft}
}

\newcommand{\epigrafe}[2]{
\clearpage
\thispagestyle{empty}
\begin{flushleft}
\vspace*{0.5\paperheight\relax}
\hspace*{0.4\paperwidth\relax}
\begin{minipage}[l]{0.5\textwidth}
``{#1}''\\#2
\end{minipage}
\end{flushleft}
}

\newenvironment{resumo}{\part*{Resumo}\thispagestyle{empty}\begin{singlespace}\noindent\normalsize}{\end{singlespace}}

\renewenvironment{abstract}{\selectlanguage{english}\part*{Abstract}\thispagestyle{empty}\begin{singlespace}\noindent\normalsize}{\end{singlespace}\selectlanguage{portuges}}

\newenvironment{agradecimentos}{\part*{Agradecimentos}\thispagestyle{empty}}{}

\RequirePackage{imakeidx}
\renewcommand{\indexname}{\'Indice}
\let\oldmakeindex\makeindex
\let\oldprintindex\printindex
\renewcommand{\makeindex}{\oldmakeindex[title=\hfill \'INDICE \hfill \mbox{}]}
\renewcommand{\printindex}{\addcontentsline{toc}{chapter}{\'INDICE}%
\renewcommand{\chapter}{%
\@startsection{chapter}{0}{0pt}{0pt}{1.5cm}{\clearpage\fontsize{12pt}{14.4pt}\bf\MakeUppercase}}%
\oldprintindex}%

\RequirePackage[pdftex,pdfborder={0 0 0},colorlinks={false}]{hyperref}

\ifsublist
  \RequirePackage[nomain,nonumberlist]{glossaries} %acronym,
  %estilo usado como base
  \setglossarystyle{alttree}
  % Configuracao de identacao do nivel 0 (titulos)
  \glssetwidest[0]{}
  % Configuracao de identacao do nivel 1 (a lista de simbolos em si)
  \glssetwidest[1]{aaaaaaaaaaaa}
\else
  \RequirePackage[nomain,nonumberlist,nogroupskip]{glossaries} %acronym,
\fi

\renewcommand*{\acronymname}{\hfill Lista de Abreviaturas \hfill \mbox{}}
\newglossary{simbolos}{sym}{sbl}{\hfill Lista de S\'imbolos \hfill \mbox{}}

\newcommand{\palavraschave}[1]{\mbox{}\\\noindent Palavras-chave: #1}% o resumo pede palavras chave no final
\newcommand{\keywords}[1]{\mbox{}\\\noindent Keywords: #1}% mesma coisa, mas pro abstract

\renewcommand{\appendix}{%
\renewcommand{\chaptername}{\appendixname}%
\setcounter{chapter}{0}% zera o contador do capítulo
\renewcommand{\thechapter}{\Alph{chapter}}% deixa o contador do capítulo em alfabético
\renewcommand{\chapter}[1]{% redefine o comando do capítulo
\stepcounter{chapter}% soma 1 ao contador do capítulo
\clearpage\thispagestyle{empty}\mbox{}\vfill\begin{center}\MakeUppercase{\textbf{AP\^ENDICE \thechapter\ --} ##1}\end{center}\vfill% adiciona uma folha com a letra e título do apêndice
\addcontentsline{toc}{chapter}{AP\^ENDICE \Alph{chapter} -- ##1}%
\newpage%
}%
}%

\newcommand{\anexos}{%
\renewcommand{\chaptername}{Anexo}%
\setcounter{chapter}{0}%
\renewcommand{\thechapter}{\Alph{chapter}}%
\renewcommand{\chapter}[1]{%
\stepcounter{chapter}%
\clearpage\thispagestyle{empty}\mbox{}\vfill\begin{center}\MakeUppercase{\textbf{ANEXO \thechapter\ --} ##1}\end{center}\vfill%
\addcontentsline{toc}{chapter}{ANEXO \Alph{chapter} -- ##1}%
\newpage%
}%
}%

\RequirePackage[alf,abnt-emphasize=bf,abnt-repeated-author-omit=yes]{abntex2cite}

\renewenvironment{quote}
               {\begin{singlespace}\list{}{%
               \fontsize{10pt}{1em}%
               \leftmargin=2cm \rightmargin=2cm}%
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
  \clearpage%
  \addcontentsline{toc}{chapter}{REFER\^ENCIAS}% adiciona o titulo ao sumario
  \oldbibliography{#1}% adiciona realmente a bibliografia
}
%% 
%% Copyright (C) 2014 by Douglas De Rizzo Meneghetti <douglasrizzo@fei.edu.br>
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
