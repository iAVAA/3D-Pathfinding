FROM nginx:1.25-alpine

RUN rm -rf /usr/share/nginx/html/*

COPY pages /usr/share/nginx/html/pages
COPY css   /usr/share/nginx/html/css
COPY js    /usr/share/nginx/html/js

RUN chmod -R 755 /usr/share/nginx/html && \
    chown -R nginx:nginx /usr/share/nginx/html

COPY nginx.conf  /etc/nginx/conf.d/default.conf

EXPOSE 80

CMD ["nginx", "-g", "daemon off;"]