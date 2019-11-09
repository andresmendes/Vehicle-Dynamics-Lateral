NAME  = vehicleArticulated
all:	text clean
text: $(NAME).tex
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(NAME).tex
	bibtex $(NAME).aux
	makeglossaries $(NAME)
	makeindex $(NAME).idx
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(NAME).tex
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(NAME).tex
	evince $(NAME).pdf &
apres: $(APRESFEI).tex
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(APRESFEI).tex
	bibtex $(APRESFEI).aux
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(APRESFEI).tex
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(APRESFEI).tex
	evince $(APRESFEI).pdf &
proj: $(PROJ).tex
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(PROJ).tex
	bibtex $(PROJ).aux
	makeglossaries $(PROJ)
	makeindex $(PROJ).idx
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(PROJ).tex
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(PROJ).tex
slides: $(APRE).tex clean
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(APRE).tex
	bibtex $(APRE).aux
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(APRE).tex
	-pdflatex -recorder -interaction=nonstopmode -shell-escape $(APRE).tex
clean:
	rm -f *.acn *.acr *.alg *.aux *.bbl *.blg *.fls *.glg *.glo *.gls *.glsdefs *.hd *.idx *.ilg *.ind *.ins *.ist *.log *.toc *.loa *.loe *.lof *.lot *.mw *.out  *.sbl *.sym *.xdy *.nav *.snm *.slg *.slo *sls
