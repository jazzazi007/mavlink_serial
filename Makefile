CC = cc
CFALGS = -Wall -Wextra
# adapt the following falgs...
#gcc -o gnc gnc.c -lSDL2 -lm
SRCS = main.c gnc.c read_autopilot.c send_autopilot.c gnc_phases.c
OBJS = $(SRCS:.c=.o)
NAME = rpi_gnc
all: $(NAME)
$(NAME): $(OBJS)
	$(CC) $(CFALGS) $(OBJS) -o $(NAME) -lSDL2 -lSDL2_ttf -lm
%.o: %.c
	$(CC) $(CFALGS) -c $< -o $@
clean:
	rm -f $(OBJS)
fclean: clean
	rm -f $(NAME)
re: fclean all
.PHONY: all clean fclean re