#ifndef twi_h
#define twi_h
/* twi.h */
/* avr.twi - TWI AVR module. {{{
 *
 * Copyright (C) 2005 Demonchy Cl�ment
 *
 * Robot APB Team/Efrei 2006.
 *        Web: http://assos.efrei.fr/robot/
 *      Email: robot AT efrei DOT fr
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * }}} */

/** Initialise twi. */
void
twi_init (uint8_t addr);

#if AC_TWI_SLAVE_ENABLE
/** R�cup�re dans buffer les donn�es recues en tant qu'esclave.
 * @return  the amout of read data.
 */
uint8_t 
twi_sl_poll (uint8_t *buffer, uint8_t size);
/** Met � jour le buffer de donn�e � envoyer */
void 
twi_sl_update (uint8_t *buffer, uint8_t size);
#endif /* AC_TWI_SLAVE_ENABLE */

#if AC_TWI_MASTER_ENABLE
/** Is the current transaction finished ? */
int8_t 
twi_ms_is_finished (void);
/** Send len bytes of data to address */
int8_t
twi_ms_send (uint8_t address, uint8_t *data, uint8_t len);
/** Read len bytes at addresse en put them in data */
int8_t
twi_ms_read (uint8_t address, uint8_t *data, uint8_t len);
#endif /* AC_TWI_MASTER_ENABLE */

#endif /* twi_h */
