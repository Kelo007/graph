while true; do
  build/generator i.json 20000 200000 10 && build/main i.json o.json && build/checker i.json o.json
  if [ $? -ne 0 ]; then
    break
  fi
done