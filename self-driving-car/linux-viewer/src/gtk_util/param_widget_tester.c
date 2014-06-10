#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <gtk/gtk.h>

#include <gtk_util/param_widget.h>

enum {
    MENU_1,
    MENU_2,
    MENU_3,
};

static void
on_param_changed( GtkuParamWidget *pw, const char *name, void *user_data )
{
    if( ! strcmp( name, "Integer 1" ) ) {
        printf("int1: %d\n", gtku_param_widget_get_int( pw, name ) );
    } else if ( ! strcmp( name, "Integer Slider" ) ) {
        printf("int slider: %d\n", gtku_param_widget_get_int( pw, name ) );
    } else if ( ! strcmp( name, "Double Slider" ) ) {
        printf("double slider: %f\n", gtku_param_widget_get_double( pw, name ) );
    } else if ( ! strcmp( name, "check1" ) ||
                ! strcmp( name, "check2" ) ||
                ! strcmp( name, "check3" ) ) {
        printf("%s: %s\n", name, 
                gtku_param_widget_get_bool( pw, name ) ? "True" : "False" );
    } else if( ! strcmp( name, "Menu" ) ) {
        printf("menu: %s / %d\n", 
                gtku_param_widget_get_enum_str( pw, name ),
                gtku_param_widget_get_enum( pw, name ) );
    }
}

int main(int argc, char **argv)
{
    printf("param_widget_test\n");

    GtkWidget *window;
    GtkuParamWidget *pw;

    gtk_init( &argc, &argv );

    window = gtk_window_new( GTK_WINDOW_TOPLEVEL );
    g_signal_connect( G_OBJECT(window), "delete_event", gtk_main_quit, NULL );
    g_signal_connect( G_OBJECT(window), "destroy", gtk_main_quit, NULL );
    gtk_container_set_border_width( GTK_CONTAINER(window), 10 );

    pw = GTKU_PARAM_WIDGET( gtku_param_widget_new() );
    gtk_container_add( GTK_CONTAINER(window), GTK_WIDGET( pw ) );

    gtku_param_widget_add_int( pw, "Integer 1", GTKU_PARAM_WIDGET_SPINBOX,
            INT_MIN, INT_MAX, 1, 10 );

    gtku_param_widget_add_int( pw, "Integer Slider", GTKU_PARAM_WIDGET_SLIDER,
            0, 100, 50, 1 );

    gtku_param_widget_add_double( pw, "Double Slider", GTKU_PARAM_WIDGET_SLIDER,
            0, 1, 0.01, 0.5 );

    gtku_param_widget_add_booleans( pw, GTKU_PARAM_WIDGET_CHECKBOX,
            "check1", 0, 
            "check2", 1, 
            "check3", 0, NULL );

    gtku_param_widget_add_enum( pw, "Menu", GTKU_PARAM_WIDGET_MENU,
            MENU_2,
            "entry 1", MENU_1,
            "entry 2", MENU_2,
            "entry 3", MENU_3,
            NULL );

    g_signal_connect( G_OBJECT( pw ), "changed", 
            G_CALLBACK( on_param_changed ), NULL );

    gtk_widget_show_all( window );
    gtk_main();

    return 0;
}
