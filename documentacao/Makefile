NAME  = documentacao
APRE  = apresentacao
all:	text clean
text: $(NAME).tex
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(NAME).tex
	bibtex $(NAME).aux
	makeglossaries $(NAME)
	makeindex $(NAME).idx
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(NAME).tex
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(NAME).tex
slides: $(APRE).tex clean
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(APRE).tex
	bibtex $(APRE).aux
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(APRE).tex
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(APRE).tex
clean:
	rm -f *.acn *.acr *.alg *.aux *.bbl *.blg *.fls *.glg *.glo *.gls *.glsdefs *.hd *.idx *.ilg *.ind *.ins *.ist *.log *.toc *.loa *.loe *.lof *.lot *.mw *.out  *.sbl *.sym *.xdy *.nav *.snm

